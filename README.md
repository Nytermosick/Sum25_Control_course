# Sum25_Control_course

**Для того, чтобы склонировать репозиторий к себе и настроить виртуальное окружение:**
1) Перейдите в удобную для вас папку
2) Пропишите команду
    ```sh
    git clone https://github.com/Nytermosick/Sum25_Control_course.git
    ```
3) Перейдите в клонированный репозиторий, затем создайте виртутальное окружение для работы:
    ```sh
    python3.10 -m venv control_env
    ```
4) Активируйте созданное виртуальное окружение:
    - **На Linux/macOS**
    ```sh
    source control_env/bin/activate
    ```
    - **На Windows (cmd)**
    ```sh
    control_env\Scripts\activate
    ```
    - **На Windows (PowerShell)**
    ```sh
    control_env\Scripts\Activate.ps1
    ```
5) Установите требуемые пакеты из `requirements.txt`:
    ```sh
    pip install -r requirements.txt
    ```