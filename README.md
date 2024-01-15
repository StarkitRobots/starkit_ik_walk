# Инструкция по развертыванию мира ходьбы

В данном файле содержится информация о всех шагах, которые стоит предпринять для разворота мира ходьбы на Вашем локальном устройстве.

## Системные требования:

Данная инструкция была протестирована на следующих конфигурациях системы:

- Win11 + WSL2: Ubuntu 20.04.6 LTS; Python: 3.11
- Ubuntu 22.04.3 LTS (dual boot);   Python: 3.10

## Запуск

Для полноценного запуска следует выполнить следующие шаги

### Шаг 1: установка webots

Для работы с миром ходьбы необходимо установить [webots](http://cyberbotics.com). 
При этом рекомендуется это делать не при помощи snap, а с использованием .deb пакета с сайта.

```bash

$ sudo apt --fix-broken install ./webots_2023b_amd64.deb

```

### Шаг 2: установка ros noetic desktop

Далее для полноценной работы необходимо установить [ros noetic desktop](https://wiki.ros.org/noetic) по [инструкции](https://wiki.ros.org/noetic/Installation/Ubuntu).

### Шаг 3: дополнительные пакеты

Во время установки ros можно выполнить установку дополнительных пакетов. Вам понадобятся [tkinter](https://wiki.python.org/moin/TkInter) и [eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page). Для этого можно использовать команды:

```bash

$ sudo apt install python3-tk
$ sudo apt install libeigen3-dev

```

### Шаг 4: сборка библиотеки для работы мира

Далее необходимо клонировать репозиторий [ik_walk](https://github.com/StarkitRobots/starkit_ik_walk) и обновить его, добавляя библиотеки [pybind11](https://github.com/pybind/pybind11) и [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). Для этого нужно выполнить команды

```bash

$ git clone ...
$ git submodules update --init --recursive

```

После этого переходим в папку с библиотекой и собираем проект, используя [CMake](https://cmake.org/)

```bash

$ cd libraries/IKWalk
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .

```

Получившийся файл с расширением .so необходимо переместить в папку controllers/ik_walk/

### Шаг 5: запуск мира

После выполнения всех действий можно перейти в папку worlds и запустить мир

```bash

$ cd worlds
$ webots sahr.wbt

```
После этого процесс установки мира можно считать завершённым