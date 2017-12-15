# Clever

Пакет программ и библиотек для Клевера.

Основная документация
---------------------

https://copterexpress.gitbooks.io/clever/content/

**Готовый образ** для RPi 3 с предустановленным и преднастроенным clever_bundle можно скачать [здесь](https://copterexpress.gitbooks.io/clever/content/docs/microsd_images.html).

[Описание API](https://copterexpress.gitbooks.io/clever/content/docs/simple_offboard.html) для автономных полетов.

Установка
---------

Склонировать репозиторий в папку `/home/pi/catkin_ws/src/clever` (**важно**):

```bash
cd ~/catkin_ws/src
git clone https://github.com/CopterExpress/clever_bundle.git clever
```

Пересобрать ROS-пакеты:

```bash
cd ~/catkin_ws
catkin_make -j1
```

Перевести submodule clever-frontend на ветку build

Включить сервис roscore (если он не включен):

```bash
sudo systemctl enable /home/pi/catkin_ws/src/clever/deploy/roscore.service
sudo systemctl start roscore
```

Включить сервис clever:

```bash
sudo systemctl enable /home/pi/catkin_ws/src/clever/deploy/clever.service
sudo systemctl start clever
```

Зависимости
-----------

[ROS Kinetic](http://wiki.ros.org/kinetic).

Необходимые для работы ROS-пакеты:

* `opencv3`
* `mavros`
* `rosbridge_suite`
* `web_video_server`
* `cv_camera`
* `nodelet`
* `dynamic_reconfigure`
* `bondcpp`, ветка `master`
* `roslint`
* `rosserial`

TODO: внести в package.xml
