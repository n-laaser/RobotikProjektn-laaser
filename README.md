# **Robotik Projekt - Repository von Nikolas Laaser**

## **Belegaufgabe 1 (Publisher Subscriber)**
### **Funktionsweise**
Aufgabe war es zwei Packages zu erstellen, welche die jeweils die Aufgabe eines Publishers und eines Subscribers übernehmen sollen.
Das Package cpp_pubsub enthält zwei ausführbare Dateien in cpp_pubsub/src, eine davon ist ein Publisher, die andere ein Subscriber für die Belegaufgabe ist jedoch nur der Publisher von nöten.
Das andere Package ist timing_tubaf_py und enthält eine ausführbare Datei listener.py, welche hier als Subscriber funktioniert.
Die beiden Packages bewirken wenn man sie korrekt mit ros2 buildet, dass publisher_member_function.cpp kontinuierlich "Hello World!" Nachrichten auf dem Topic "number"versendent
und der listener.py hört den Kanal ab und gibt das Hello WOrld mit der jeweiligen Zahl wieder. ZUsätzlich erstellt listener.py ein weiteres Topic Namens "diff" und veröffentlicht dort
die Zeitdifferenz zwischen den letzten beiden Nachrichten die durch "number" angekommen sind.

### **Abhängigkeiten**
Um die Packages nutzen zu können, muss zuerst ROS2 Humble heruntergeladen werden und ein Workspace eingerichtet werden. 
Zum Testen kann man einmal ein Demo pubsub programm ausführen, um sicherzustellen, dass alles korrekt funktioniert.
Abhängigkeiten die in ROS für diese 2 Packages benötigt werden sind:

- **rclpy**: Für die ROS 2-Python-API und die ROS 2-C++ API und die Erstellung von Nodes, Publishern und Subscribern.
- **std_msgs**: Für den Nachrichtentyp `String`, der im Subscriber verwendet wird.
- **builtin_interfaces**: Für den Nachrichtentyp `Duration`, der für die Zeitdifferenzberechnung und Veröffentlichung genutzt wird.
Man kann die Abhängigkeiten installieren indem man in der Kommandozeile in Linux (bei mir Ubuntu) eingibt:
sudo apt update
sudo apt install ros-<distro>-rclpy ros-<distro>-std-msgs ros-<distro>-builtin-interfaces
Außerdem werden <chrono> <functional> <memory> und <string> von dem .cpp Package verwendwet, sind aber teil der C++ Standardbibliothek.

Man kann auch in ~/ros2_ws den Befehl
rosdep install -i --from-path src --rosdistro humble -y
ausführen um nach nach fehlenden Abhängigkeiten zu prüfen

### **Startanleitung**

Öffne ein Terminal
Begib dich in die root deines ROS2 Workspaces, wahrscheinlich ~/ros2_ws 
"cd ~/ros2_ws"
danach stelle sicher dass du die Packages richtig in deinem ros2_ws/src ordner heruntergeladen hast. 
"cd src" um in die src directory zu gehen
"ls" um dir den inhalt anzeigen zu lassen
Wenn dort die Packages cpp_pubsub und timing_tubaf_py zu sehen sind, kannst du sie verwenden.

Gehe wieder in den root des Workspaces( "cd ~/ros2_ws")
Danach Builde das C++ Package mit: "colcon build --packages-select cpp_pubsub"
Das Pyhton Package kannst du mit: "colcon build --packages-select timing_tubaf_py" builden.

#### Publisher starten
Öffne ein Neues terminal und führe erst ein setup durch:
". install/setup.bash"
Im /src ordner des Package cpp_pubsub sind 2 .cpp dateien enthalten, aber wir brauchen nur "publisher_member_function.cpp" da dieses Dokument 
den Code enthält der auf dem Topic "number" Nachrichten veröffentlicht. In dem CMakeLists.txt datei ist festgehalten dass man "talker" aufrufen muss anstatt
jedes mal den langen namen der .cpp datei einzugeben.
Das heißt du kannst den Publisher aufrufen durch
"ros2 run cpp_pubsub talker"
und nun sollte immer wieder "Hello World! 1" angezeigt werden (bloß dass die nummer immer größer wird).

### Subscriber starten
Öffne nun noch ein neues Terminal und führe erst den Befehel:
". install/setup.bash" 
aus und dann den Befehl:
"ros2 run timing_tubaf_py listener"

Nun sollte in dem einen Terminal die Hello World nachrichten gesendet werden und in dem Anderen Terminal ausgegeben werden, 
inklusive der zeitlichen Verzögerung zwischen den letzten zwei Nachrichten.
Wenn man nun, während die beiden programme laufen, noch ein Terminal öffnet und dort "ros2 topic list" eingibt, kann man sehen Welche topics gerade offen sind.
Man sollte die topics /diff und /number sehen, jedoch nur wenn die beiden Programme noch laufen.
