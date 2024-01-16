# Инструкция по развертыванию мира ходьбы

В данном файле содержится информация о всех шагах, которые стоит предпринять для разворота мира ходьбы на Вашем локальном устройстве.

## Системные требования:

Данная инструкция была протестирована на следующих конфигурациях системы:

- Win11 + WSL2: Ubuntu 20.04.6 LTS; Base Python: 3.11
- Ubuntu 22.04.3 LTS (dual boot);   Base Python: 3.10

## Содержание

- [Шаг 1: предварительные действия](#шаг-1-предварительные-действия)
    - [Git](#git)
    - [Python](#python)
    - [Система сборки](#система-сборки)
- [Шаг 2: установка webots](#шаг-2-установка-webots)
    - [Вариант 1: простая установка](#вариант-1-простая-установка)
    - [Вариант 2: RoboCup Humanoid webots](#вариант-2-robocup-humanoid-webots)
- [Шаг 3: сборка библиотеки для работы мира](#шаг-3-сборка-библиотеки-для-работы-мира)
- [Шаг 4: запуск мира](#шаг-4-запуск-мира)
- [Дополнительно (решение возможных проблем при установке)](#дополнительно-решение-возможных-проблем-при-установке)
    - [Дополнительные пакеты](#дополнительные-пакеты)
    - [Установка ROS](#установка-ros)
        - [WSL](#wsl)
        - [Dual boot](#dual-boot)

## Шаг 1: предварительные действия

### Git

Сперва стоит проверить, установлен ли Git на компьютере. Для этого можно ввести в консоль команду 

```bash
git --version
```

Если в выводе не будет указана установленная версия Git, то надо выполнить действия, описанные в инструкции на [сайте Git](https://git-scm.com/download/linux).

---

### Python

Для дальнейшей работы с миром ходьбы необходим установленный интерпретатор Python 3. Проверить текущую версию интерпретатора можно, введя в консоль команду:

```bash
python3 --version
```

Если интерпретатор не установлен, то следует воспользовать инструкцей по установке. Можно рассмотреть один из вариантов, представленных в [этой статье](https://phoenixnap.com/kb/how-to-install-python-3-ubuntu). Лучше всего рассмотреть вариант с [установкой Python 3 версий 3.6-3.9](https://www.makeuseof.com/install-python-ubuntu/), что может быть полезным для дальнейшей установки.

Кроме того, стоит заранее установить **python3-dev** и **tkinter**:
```bash
sudo apt install python3-dev
sudo apt install python3-tk
```

---

### Система сборки

Для сборки библиотеки понадобится также система сборки [CMake](https://cmake.org/). Проверить её версию можно, используя команду консоли:

```bash
cmake --version
```

Если же система не установлена, то можно выполнить команду

```bash
sudo apt install cmake
```

чтобы установить последнюю версию [CMake](https://cmake.org/). Можно также воспользоваться и другими методами установки, если это необходимо.


## Шаг 2: установка webots

Для работы с миром ходьбы необходимо установить [webots](http://cyberbotics.com). 

Тут есть несколько возможных вариантов

### Вариант 1: простая установка

Данный способ достаточно простой. В результате на устройстве будет рабочая последняя версия webots, готовая для запуска с контроллерами из настоящей библиотеки.

Первым делом необходимо скачать пакет **webots_xxxxx_amd64.deb** с официального сайта [webots](http://cyberbotics.com). После этого переходим в папку со скачанным пакетом и устанавливаем его командой 

```bash
sudo apt --fix-broken install ./webots_2023b_amd64.deb
```

---

### Вариант 2: RoboCup Humanoid webots

Этот вариант представляет интерес из-за некоторых возможностей, которые не поддерживаются текущей версие webots. Например, в данной версии webots поддерживается люфт, что может быть полезно в некоторых задачах.

Для полноценной установки необходимо клонировать исходники [из репозитория с webots'ом RoboCup Humanoid League TC](https://github.com/RoboCup-Humanoid-TC/webots) и далее следовать [официальной инструкции](https://github.com/cyberbotics/webots/wiki/Linux-installation/). Начинать установку можно с команды

```bash
git clone --recurse-submodules -j8 https://github.com/RoboCup-Humanoid-TC/webots.git
cd webots
```

Перед сборкой стоит дополнительно установить python3.6 - python3.9, если текущая версия новее или старее - это нужно для корректной работы контроллера. Если сборка проводилась с другими версиями Python, то необходимо пересобрать webots из того же репозитория.

Кроме того, в случае, если производилась установка другой версии интерпретатора, то стоит переключиться на неё, так как далее необходимо будет собирать библиотеку с использованием именно этой версии Python. О том, как управлять различными версиями интерпретатора Python3, можно почитать [тут](https://hackersandslackers.com/multiple-python-versions-ubuntu-20-04/). 

После переключения на нужную версию установим несколько дополнительных пакетов:

```bash
sudo apt install python3.x-dev          # to make webots
sudo apt install python3.x-distutils    # to make IKWalk 
sudo apt install python3.x-tk           # to run controller
```

Тут вместо python3.x надо подставить свою версию интерпретатора.

Новая установленная версия Python ещё понадобится для корректной настройки webots (см. [шаг 4](#шаг-4-запуск-мира))

## Шаг 3: сборка библиотеки для работы мира

Далее необходимо клонировать репозиторий [ik_walk](https://github.com/StarkitRobots/starkit_ik_walk) и обновить его, добавляя библиотеки [pybind11](https://github.com/pybind/pybind11) и [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). Для этого нужно выполнить команды

```bash
git clone https://github.com/StarkitRobots/starkit_ik_walk.git
cd starkit_ik_walk
git submodule update --init --recursive
```

Теперь в директориях libraries/eigen и libraries/pybind11 появятся файлы этих зависимостей, которые будут использованы в сборке проекта.

Чтобы получить бинарный файл с библиотекой нужно собрать проект и скопировать результат в директорию с контроллером. Для этого, используя [CMake](https://cmake.org/), получим билд библиотеки, выполнив следующие команды:

```bash
cd libraries/IKWalk
mkdir build
cd build
cmake ..
cmake --build .
cp starkit_ik_walk.cpython-3xx-x86_64-linux-gnu.so ../../../controllers/ik_walk/ # instead of 3xx need to place your version of python3; ex: 310 for Python 3.10.xx
```

## Шаг 4: запуск мира

После выполнения всех действий можно перейти в папку worlds и запустить мир

```bash
cd worlds
webots sahr.wbt
```

Если производилась установка webots версии из репозитория RoboCup Humanoid, то для начала стоит убедиться, что используются правильные файлы мира и proto-файл. 

Для запуска стоит зайти в директорию с бинарным фалом webots и запустить соответствующий мир:

```bash
./webots ~/path_to_starkit_ik_walk/starkit_ik_walk/worlds/sahr_2021.wbt
```

Стоит отметить, что можно изменить версию интерпретатора Python для запуска контроллера через интерфейс webots. Для этого на верхней панели выберите меню **Tools&rarr;Preferences...**. Далее во вкладке **General** необходима строка **Python command**: вписываем туда команду для той версии Python, с которой компилировался webots (например, если компилировался с python3.9, то вписываем команду python3.9). Нажимаем "Ок" и перезагружаем мир. Однако этот пункт **не является необходимым**, особенно если вы переключили версию интерпретатора при помощи **update-alternatives**

## Дополнительно (решение возможных проблем при установке)

В данном разделе содержатся пункты, которые могут быть полезными в различных ситуациях и могут служить решением некоторых проблем.

### Дополнительные пакеты
В некоторых случая стоит помнить о необходимости установить дополнительные пакеты. Могут понадобятся следующие пакеты:

```bash
sudo apt install python3-tk
sudo apt install python3-dev
sudo apt install python3-distutils
sudo apt install libeigen3-dev
```

### Установка ROS

Кроме того, для полноценной работы необходим ROS - Robot Operating System - набор программных средств для разработки програмного обеспечения для робота.

Здесь возможно несколько вариантов:

#### WSL

В случае работы с WSL можно установить [ros noetic desktop](https://wiki.ros.org/noetic). Его можно установить по [официальной инструкции](https://wiki.ros.org/noetic/Installation/Ubuntu).

---

#### Dual boot

Если основной операционной системой является Linux или он установлен в качестве второй ОС, то стоит установить [ROS2](https://www.ros.org/). Например, [ros-humble-desktop](https://docs.ros.org/en/humble/Installation.html), используя официальную инструкцию.

К этой инструкции стоит добавить, что можно указать источник ROS2 setup.bash в .bashrc, чтобы ROS2 был доступен при каждом запуске терминала.

Для этого стоит выаполнить команду 

```bash
echo "source /opt/ros/humble/source.bash" >> ~/.bashrc
```

После этого можно обновить консоль. Кроме того, при каждом открытии можно свободно использовать ROS2.
