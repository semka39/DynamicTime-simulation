using SDL2;
using static System.Net.Mime.MediaTypeNames;

namespace SOE
{
    internal class Program
    {
        private static readonly PriorityQueue<Bit, float> Bits = new();
        private static float _currentTime = 0;
        private static float _currentStep = 0;
        private static int _width, _height;
        private static readonly Random Rnd = new();

        private static int RenderSpeed = 1000, SimulationSpeed = 1;
        private const float GravitationalConst = 15;

        private static float _offsetX, _offsetY, _scale = 1;

        // SDL Objects
        private static IntPtr _window;
        private static IntPtr _renderer;

        static void Main(string[] args)
        {
            InitializeSDL();
            InitializeBits(100);
            ProcessInteractions();
            RunSimulation();
        }

        private static void InitializeSDL()
        {
            SDL.SDL_Init(SDL.SDL_INIT_VIDEO);
            _window = SDL.SDL_CreateWindow("Simulation", SDL.SDL_WINDOWPOS_UNDEFINED, SDL.SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL.SDL_WindowFlags.SDL_WINDOW_RESIZABLE);
            _renderer = SDL.SDL_CreateRenderer(_window, -1, SDL.SDL_RendererFlags.SDL_RENDERER_ACCELERATED);
        }

        private static void InitializeBits(int count)
        {
            for (int i = 0; i < count; i++)
            {
                int mass = Rnd.Next(1, 15);
                Bit bit = new(mass, mass * 2, new Point(Rnd.Next(-15000, 15000), Rnd.Next(-15000, 15000)));

                float forceScale = Rnd.Next(10);
                float forceAngle = (float)Rnd.Next((int)Math.Round(2 * Math.PI * 10000)) / 10000;
                bit.AddForce(new Vector(forceScale, forceAngle));

                Bits.Enqueue(bit, 0);
            }
        }

        private static float CalculateForce(Bit bit1, Bit bit2, float distanceSquared, float combinedRadius)
        {
            int direction = distanceSquared > combinedRadius * combinedRadius ? -1 : 1;
            return (bit1.Mass * bit2.Mass * GravitationalConst) / distanceSquared * direction;
        }

        private static void RunSimulation()
        {
            float lastIterationTime = 0;
            bool running = true;

            while (running)
            {
                while (SDL.SDL_PollEvent(out SDL.SDL_Event e) != 0)
                {
                    switch (e.type)
                    {
                        case SDL.SDL_EventType.SDL_QUIT:
                            running = false;
                            break;
                        case SDL.SDL_EventType.SDL_MOUSEWHEEL:
                            const float wheelstep = 1.5f;
                            float wheelmul = 1;
                            if (e.wheel.y > 0)
                                wheelmul = wheelstep;
                            else if (e.wheel.y < 0)
                                wheelmul = 1 / wheelstep;

                            int mouseX, mouseY;
                            SDL.SDL_GetMouseState(out mouseX, out mouseY);

                            float worldX = (mouseX - _width / 2 - _offsetX) / _scale;
                            float worldY = (mouseY - _height / 2 - _offsetY) / _scale;

                            _scale *= wheelmul;

                            _offsetX = mouseX - _width / 2 - worldX * _scale;
                            _offsetY = mouseY - _height / 2 - worldY * _scale;
                            break;
                        case SDL.SDL_EventType.SDL_KEYDOWN:
                            const int step = 20;
                            switch (e.key.keysym.sym)
                            {
                                case SDL.SDL_Keycode.SDLK_w:
                                    _offsetY += step;
                                    break;
                                case SDL.SDL_Keycode.SDLK_s:
                                    _offsetY -= step;
                                    break;
                                case SDL.SDL_Keycode.SDLK_d:
                                    _offsetX -= step;
                                    break;
                                case SDL.SDL_Keycode.SDLK_a:
                                    _offsetX += step;
                                    break;

                                case SDL.SDL_Keycode.SDLK_UP:
                                    RenderSpeed += 50;
                                    break;
                                case SDL.SDL_Keycode.SDLK_DOWN:
                                    RenderSpeed -= 50;
                                    if (RenderSpeed < 1)
                                        RenderSpeed = 1;
                                    break;

                                case SDL.SDL_Keycode.SDLK_RIGHT:
                                    SimulationSpeed += 1;
                                    break;
                                case SDL.SDL_Keycode.SDLK_LEFT:
                                    SimulationSpeed -= 1;
                                    if (SimulationSpeed < 1)
                                        SimulationSpeed = 1;
                                    break;
                            }
                            break;
                    }
                }

                Bits.TryDequeue(out Bit currentBit, out _currentTime);

                ProcessInteractions(currentBit);
                currentBit.Activate(_currentTime - currentBit.LastActivationTime);

                float deltaTime = SimulationSpeed / currentBit.Velocity.Scale;
                Bits.Enqueue(currentBit, deltaTime + _currentTime);

                bool render = _currentStep % RenderSpeed == 0;
                Graph.AddValue(_currentTime - lastIterationTime);
                if (render) RenderScene();

                lastIterationTime = _currentTime;
                _currentStep++;
            }
        }

        private static void RenderScene()
        {
            SDL.SDL_GetWindowSize(_window, out _width, out _height);
            SDL.SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
            SDL.SDL_RenderClear(_renderer);

            foreach (var bitItem in Bits.UnorderedItems)
            {
                Bit bit = bitItem.Element;
                DrawBit(_renderer, bit);
            }

            Graph.Draw(_renderer);
            DisplayText(new IPoint(10, 70), 2, 3, $"render speed     - {RenderSpeed}");
            DisplayText(new IPoint(10, 90), 2, 3, $"simulation speed - {SimulationSpeed}");
            DisplayText(new IPoint(10, 110), 2, 3, $"scale            - {_scale}");
            DisplayText(new IPoint(10, 130), 2, 3, $"time             - {_currentTime}");
            DisplayText(new IPoint(10, 150), 2, 3, $"step             - {_currentStep}");

            SDL.SDL_RenderPresent(_renderer);
        }

        private static void ProcessInteractions()
        {
            foreach (var bitItem1 in Bits.UnorderedItems)
            {
                Bit bit1 = bitItem1.Element;

                ProcessInteractions(bit1);

                bit1.Activate(1);
            }
        }

        private static void ProcessInteractions(Bit currentBit)
        {
            foreach (var bitItem in Bits.UnorderedItems)
            {
                Bit otherBit = bitItem.Element;

                if (currentBit == otherBit) continue;

                float dx = currentBit.Position.x - otherBit.Position.x;
                float dy = currentBit.Position.y - otherBit.Position.y;
                float distanceSquared = dx * dx + dy * dy;

                if (distanceSquared == 0) continue;

                float combinedRadius = currentBit.Radius + otherBit.Radius;
                float forceMagnitude = CalculateForce(currentBit, otherBit, distanceSquared, combinedRadius);

                float angle = (float)Math.Atan2(dy, dx);
                currentBit.AddForce(new Vector(forceMagnitude, angle));
            }
        }

        static void DisplayText(IPoint cpos, int size, int interval, string display)
        {
            void Interp(string cmds, int pos)
            {
                int s = pos * interval * size;

                int[] cords = Array.ConvertAll(cmds.ToCharArray(), x => int.Parse(x.ToString()) * size);
                for (int i = 2; i < cords.Length; i += 2)
                {
                    SDL.SDL_RenderDrawLine(_renderer, cords[i - 2] + s + cpos.x, cords[i - 1] + cpos.y, cords[i] + s + cpos.x, cords[i + 1] + cpos.y);
                }
            }

            string text = display.ToLower();
            for (int i = 0; i < text.Length; i++)
            {
                switch (text[i])
                {
                    case '0':
                        Interp("0004242000", i);
                        break;
                    case '1':
                        Interp("141001", i);
                        break;
                    case '2':
                        Interp("011021220424", i);
                        break;
                    case '3':
                        Interp("001021120212231404", i);
                        break;
                    case '4':
                        Interp("0002222420", i);
                        break;
                    case '5':
                        Interp("20000112222404", i);
                        break;
                    case '6':
                        Interp("200004242202", i);
                        break;
                    case '7':
                        Interp("0020211214", i);
                        break;
                    case '8':
                        Interp("002021030424230100", i);
                        break;
                    case '9':
                        Interp("042420000222", i);
                        break;
                    case 'a':
                        Interp("04011021242202", i);
                        break;
                    case 'b':
                        Interp("00102112021223140400", i);
                        break;
                    case 'c':
                        Interp("20000424", i);
                        break;
                    case 'd':
                        Interp("00041423211000", i);
                        break;
                    case 'e':
                        Interp("20000222020424", i);
                        break;
                    case 'f':
                        Interp("040222020020", i);
                        break;
                    case 'g':
                        Interp("200004242212", i);
                        break;
                    case 'h':
                        Interp("000402222024", i);
                        break;
                    case 'i':
                        Interp("002010140424", i);
                        break;
                    case 'j':
                        Interp("002010140403", i);
                        break;
                    case 'k':
                        Interp("00040212201224", i);
                        break;
                    case 'l':
                        Interp("000424", i);
                        break;
                    case 'm':
                        Interp("0400122024", i);
                        break;
                    case 'n':
                        Interp("04002420", i);
                        break;
                    case 'o':
                        Interp("0004242000", i);
                        break;
                    case 'p':
                        Interp("040010211202", i);
                        break;
                    case 'q':
                        Interp("242010011222", i);
                        break;
                    case 'r':
                        Interp("040010211202122324", i);
                        break;
                    case 's':
                        Interp("211001231403", i);
                        break;
                    case 't':
                        Interp("14100020", i);
                        break;
                    case 'u':
                        Interp("00042420", i);
                        break;
                    case 'v':
                        Interp("001420", i);
                        break;
                    case 'w':
                        Interp("0004122420", i);
                        break;
                    case 'x':
                        Interp("0024122004", i);
                        break;
                    case 'y':
                        Interp("04201200", i);
                        break;
                    case 'z':
                        Interp("00200424", i);
                        break;
                    case '-':
                        Interp("0222", i);
                        break;
                    case ',':
                        Interp("1314", i);
                        break;
                    default:
                        break;
                }
            }
            return;
        }

        static void DrawBit(IntPtr renderer, Bit bit)
        {
            // Масштабируем центр и радиус
            int centerX = (int)(bit.Position.x * _scale) + (int)_offsetX + (_width / 2);
            int centerY = (int)(bit.Position.y * _scale) + (int)_offsetY + (_height / 2);
            int radius = (int)(bit.Radius * _scale);

            bool xInScreen = centerX + radius > 0 && centerX - radius < _width;
            bool yInScreen = centerY + radius > 0 && centerY - radius < _height;

            if (!xInScreen || !yInScreen)
                return;

            SDL.SDL_SetRenderDrawColor(renderer, bit.colorR, bit.colorG, bit.colorB, 255);

            for (int w = 0; w < radius * 2; w++)
            {
                for (int h = 0; h < radius * 2; h++)
                {
                    int dx = radius - w;
                    int dy = radius - h;
                    if ((dx * dx + dy * dy) <= (radius * radius))
                    {
                        SDL.SDL_RenderDrawPoint(renderer, centerX + dx, centerY + dy);
                    }
                }
            }

            // Масштабируем направление скорости
            Vector LastVelocity = bit.Velocity + (bit.LastForce / bit.Mass * -1);
            int velocityEndX = centerX + (int)(LastVelocity.Scale * 5 * Math.Cos(bit.Velocity.Angle) * _scale);
            int velocityEndY = centerY + (int)(LastVelocity.Scale * 5 * Math.Sin(bit.Velocity.Angle) * _scale);
            SDL.SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // Зелёный
            SDL.SDL_RenderDrawLine(renderer, centerX, centerY, velocityEndX, velocityEndY);

            // Масштабируем направление силы
            int forceEndX = velocityEndX + (int)(bit.LastForce.Scale * 5 * Math.Cos(bit.LastForce.Angle) * _scale);
            int forceEndY = velocityEndY + (int)(bit.LastForce.Scale * 5 * Math.Sin(bit.LastForce.Angle) * _scale);
            SDL.SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // Синий
            SDL.SDL_RenderDrawLine(renderer, velocityEndX, velocityEndY, forceEndX, forceEndY);
        }

        struct Vector
        {
            public float Scale, Angle;

            // Конструктор
            public Vector(float scale, float angle)
            {
                this.Scale = scale;
                this.Angle = angle;
            }

            // Перегрузка оператора сложения
            public static Vector operator +(Vector v1, Vector v2)
            {
                float angle1Rad = v1.Angle;
                float angle2Rad = v2.Angle;

                float x1 = v1.Scale * (float)Math.Cos(angle1Rad);
                float y1 = v1.Scale * (float)Math.Sin(angle1Rad);
                float x2 = v2.Scale * (float)Math.Cos(angle2Rad);
                float y2 = v2.Scale * (float)Math.Sin(angle2Rad);

                float xResult = x1 + x2;
                float yResult = y1 + y2;

                float resultantScale = (float)Math.Sqrt(xResult * xResult + yResult * yResult);
                float resultantAngle = (float)Math.Atan2(yResult, xResult);

                return new Vector(resultantScale, resultantAngle);
            }

            public static Vector operator /(Vector v, float scalar)
            {
                if (scalar == 0)
                    throw new DivideByZeroException("Деление на ноль невозможно.");

                return new Vector(v.Scale / scalar, v.Angle);
            }

            public static Vector operator *(Vector v, float scalar)
            {
                return new Vector(v.Scale * scalar, v.Angle);
            }

            public override string ToString()
            {
                return $"Vector(scale: {Scale}, angle: {Angle} rad)";
            }
        }

        struct Point
        {
            public float x, y;
            public Point(float x, float y)
            {
                this.x = x;
                this.y = y;
            }
        }

        struct IPoint
        {
            public int x, y;
            public IPoint(int x, int y)
            {
                this.x = x;
                this.y = y;
            }
        }

        class Bit
        {
            public float Mass;
            public float Radius;
            public Point Position;
            public float LastActivationTime = 0;
            public byte colorR, colorG, colorB;

            public Vector Force, LastForce, Velocity;

            public Bit(float mass, float radius, Point position)
            {
                Mass = mass;
                Radius = radius;
                Position = position;
                colorR = (byte)Rnd.Next(255);
                colorG = (byte)Rnd.Next(255);
                colorB = (byte)Rnd.Next(255);
            }

            public void Activate(float deltaTime)
            {
                Position.x += Velocity.Scale * deltaTime * (float)Math.Cos(Velocity.Angle);
                Position.y += Velocity.Scale * deltaTime * (float)Math.Sin(Velocity.Angle);

                Velocity += Force * deltaTime / Mass;
                LastForce = Force;
                Force = new Vector(0, 0);
                LastActivationTime = _currentTime;
            }

            public void AddForce(Vector force)
            {
                Force += force;
            }
        }

        static class Graph
        {
            private static readonly List<float> values = new List<float>();
            private const int GraphHeight = 50; // Высота графика
            private const int OffsetX = 10;  // Смещение по Y
            private const int OffsetY = 10;  // Смещение по Y

            public static void AddValue(float value)
            {
                // Добавляем новое значение в список
                values.Add(value);

                // Удаляем старые значения, если их больше MaxValues
                if (values.Count > _width - OffsetX * 2)
                {
                    values.RemoveAt(0);
                }
            }

            public static void Draw(IntPtr renderer)
            {
                if (values.Count < 2) return;

                // Определяем максимальное значение для нормализации
                float maxValue = values.Max();
                if (maxValue == 0) maxValue = 1; // Избегаем деления на ноль

                // Устанавливаем цвет графика (например, белый)
                SDL.SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

                // Рисуем график
                for (int i = 1; i < values.Count; i++)
                {
                    // Нормализуем значения для отображения на графике
                    float normalizedPrev = values[i - 1] / maxValue * GraphHeight;
                    float normalizedCurr = values[i] / maxValue * GraphHeight;

                    // Рассчитываем позиции точек
                    int x1 = _width - OffsetX - (i - 1);
                    int y1 = OffsetY + GraphHeight - (int)normalizedPrev;
                    int x2 = _width - OffsetX - i;
                    int y2 = OffsetY + GraphHeight - (int)normalizedCurr;

                    // Рисуем линию между двумя точками
                    SDL.SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
                }
            }
        }
    }
}