using SDL2;

namespace SOE
{
    internal class Program
    {
        static PriorityQueue<Bit, float> bits = new PriorityQueue<Bit, float>();

        static float curentTime = 0, currentstep = 0;
        static Random rnd = new Random();

        // Инициализация SDL
        static IntPtr window;
        static IntPtr renderer;

        // Функция для отрисовки битов
        static void DrawBit(IntPtr renderer, Bit bit)
        {
            int centerX = (int)bit.position.x + 100; // Центр круга по X
            int centerY = (int)bit.position.y + 100; // Центр круга по Y
            int radius = (int)bit.radius;           // Радиус круга

            // Цвет круга (например, красный)
            SDL.SDL_SetRenderDrawColor(renderer, bit.colorR, bit.colorG, bit.colorB, 255);

            // Рисуем круг
            for (int w = 0; w < radius * 2; w++)
            {
                for (int h = 0; h < radius * 2; h++)
                {
                    int dx = radius - w; // смещение по оси X
                    int dy = radius - h; // смещение по оси Y
                    if ((dx * dx + dy * dy) <= (radius * radius)) // проверка внутри круга
                    {
                        SDL.SDL_RenderDrawPoint(renderer, centerX + dx, centerY + dy);
                    }
                }
            }

            // Рисуем направление скорости (зелёная линия)
            int velocityEndX = centerX + (int)(bit.Velocity.scale * 5 * Math.Cos(bit.Velocity.angle));
            int velocityEndY = centerY + (int)(bit.Velocity.scale * 5 * Math.Sin(bit.Velocity.angle));
            SDL.SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // Зелёный
            SDL.SDL_RenderDrawLine(renderer, centerX, centerY, velocityEndX, velocityEndY);

            // Рисуем направление силы (синяя линия)
            int forceEndX = centerX + (int)(bit.LastForce.scale * 5 * Math.Cos(bit.LastForce.angle));
            int forceEndY = centerY + (int)(bit.LastForce.scale * 5 * Math.Sin(bit.LastForce.angle));
            SDL.SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // Синий
            SDL.SDL_RenderDrawLine(renderer, centerX, centerY, forceEndX, forceEndY);
        }



        static void Main(string[] args)
        {
            SDL.SDL_Init(SDL.SDL_INIT_VIDEO);
            window = SDL.SDL_CreateWindow("Simulation", SDL.SDL_WINDOWPOS_UNDEFINED, SDL.SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL.SDL_WindowFlags.SDL_WINDOW_RESIZABLE);
            renderer = SDL.SDL_CreateRenderer(window, -1, SDL.SDL_RendererFlags.SDL_RENDERER_ACCELERATED);

            for (int i = 0; i < 30; i++)
            {
                int mass = rnd.Next(1, 15);
                Bit bit = new Bit(mass, mass * 2, new Point(rnd.Next(0, 1000), rnd.Next(0, 500)));

                float Vscale = (float)rnd.NextDouble();
                float Vangel = (float)rnd.Next((int)Math.Round(2 * Math.PI * 10000)) / 10000;
                bit.AddImpulse(new Vector(Vscale, Vangel));
                if (i == 5)
                    bit.AddImpulse(new Vector(60, Vangel));

                bits.Enqueue(bit, 0);
            }

            //просчитываем для всех взаимодействие 1 раз
            foreach (var ewithbit1 in bits.UnorderedItems)
            {
                Bit cbit = ewithbit1.Element;
                foreach (var ewithbit2 in bits.UnorderedItems)
                {
                    Bit bit2 = ewithbit2.Element;

                    float dx = cbit.position.x - bit2.position.x;
                    float dy = cbit.position.y - bit2.position.y;
                    float sumr2 = (float)Math.Pow(cbit.radius + bit2.radius, 2);
                    float distance2 = dx * dx + dy * dy;

                    if (distance2 == 0)
                        continue;

                    float angle = (float)Math.Atan2(dy, dx);

                    int direction = distance2 > sumr2 ? -100 : 1;

                    float force = (cbit.mass * bit2.mass) / distance2 * direction;

                    // Применяем импульс отталкивания
                    cbit.AddImpulse(new Vector(force * cbit.mass, angle));

                }
                cbit.Activate(1);
            }

            float lastItTime = 0;
            bool running = true;
            while (running)
            {
                bool render = currentstep % 100 == 0;

                while (SDL.SDL_PollEvent(out SDL.SDL_Event e) != 0)
                {
                    if (e.type == SDL.SDL_EventType.SDL_QUIT)
                        running = false;
                }

                bits.TryDequeue(out Bit cbit, out curentTime);

                if (render)
                {
                    SDL.SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                    SDL.SDL_RenderClear(renderer);
                    DrawBit(renderer, cbit);
                }

                //взаимодействия
                foreach (var ewithbit in bits.UnorderedItems)
                {
                    Bit bit2 = ewithbit.Element;
                    if (render)
                        DrawBit(renderer, bit2);

                    float dx = cbit.position.x - bit2.position.x;
                    float dy = cbit.position.y - bit2.position.y;
                    float sumr2 = (float)Math.Pow(cbit.radius + bit2.radius, 2);
                    float distance2 = dx * dx + dy * dy;

                    if (distance2 == 0)
                        continue;

                    float angle = (float)Math.Atan2(dy, dx);

                    int direction = distance2 > sumr2 ? -1 : 1;

                    float force = (cbit.mass * bit2.mass) * 100 / distance2 * direction;

                    // Применяем импульс отталкивания
                    cbit.AddImpulse(new Vector(force * cbit.mass, angle));

                }
                //взаимодействия
                if (render)
                {
                    Graph.AddValue(curentTime - lastItTime);
                    Graph.Draw(renderer);
                    SDL.SDL_RenderPresent(renderer);
                }

                cbit.Activate(curentTime - cbit.lastActTime);

                float dt = 1 / cbit.Velocity.scale;

                bits.Enqueue(cbit, dt + curentTime);

                lastItTime = curentTime;
                currentstep++;
            }
        }

        struct Vector
        {
            public float scale, angle;

            // Конструктор
            public Vector(float scale, float angle)
            {
                this.scale = scale;
                this.angle = angle;
            }

            // Перегрузка оператора сложения
            public static Vector operator +(Vector v1, Vector v2)
            {
                float angle1Rad = v1.angle;
                float angle2Rad = v2.angle;

                float x1 = v1.scale * (float)Math.Cos(angle1Rad);
                float y1 = v1.scale * (float)Math.Sin(angle1Rad);
                float x2 = v2.scale * (float)Math.Cos(angle2Rad);
                float y2 = v2.scale * (float)Math.Sin(angle2Rad);

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

                return new Vector(v.scale / scalar, v.angle);
            }

            public static Vector operator *(Vector v, float scalar)
            {
                return new Vector(v.scale * scalar, v.angle);
            }

            public override string ToString()
            {
                return $"Vector(scale: {scale}, angle: {angle} rad)";
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

        class Bit
        {
            public float mass;
            public float radius;
            public Point position;
            public float lastActTime = 0;
            public byte colorR, colorG, colorB;

            public Vector Force, LastForce, Velocity;

            public Bit(float mass, float radius, Point position)
            {
                this.mass = mass;
                this.radius = radius;
                this.position = position;
                colorR = (byte)rnd.Next(255);
                colorG = (byte)rnd.Next(255);
                colorB = (byte)rnd.Next(255);
            }

            public void Activate(float deltaTime)
            {
                position.x += Velocity.scale * deltaTime * (float)Math.Cos(Velocity.angle);
                position.y += Velocity.scale * deltaTime * (float)Math.Sin(Velocity.angle);

                Velocity += Force * deltaTime;
                LastForce = Force;
                Force = new Vector(0, 0);
                lastActTime = curentTime;
            }

            public void AddImpulse(Vector impulse)
            {
                Force += impulse / mass;

            }
        }
    }

    static class Graph
    {
        private static readonly List<float> values = new List<float>();
        private const int MaxValues = 1000; // Максимальное количество значений
        private const int GraphHeight = 50; // Высота графика
        private const int OffsetX = 1000; // Смещение по X (правый верхний угол)
        private const int OffsetY = 40;  // Смещение по Y

        public static void AddValue(float value)
        {
            // Добавляем новое значение в список
            values.Add(value);

            // Удаляем старые значения, если их больше MaxValues
            if (values.Count > MaxValues)
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
                int x1 = OffsetX - (i - 1);
                int y1 = OffsetY + GraphHeight - (int)normalizedPrev;
                int x2 = OffsetX - i;
                int y2 = OffsetY + GraphHeight - (int)normalizedCurr;

                // Рисуем линию между двумя точками
                SDL.SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
            }
        }
    }

}
