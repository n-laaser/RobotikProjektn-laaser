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

## **Belegaufgabe 2 (Objekt mit Laserscanner verfolgen)**
Für diese Aufgabe sollte der 'Burger' Roboter (ein kleiner Roboter mit Rädern, einem Laserscanner und einer Kamera) mithilfe des Laserscanners dem nächsten Objekt, welches sich vor ihm befindet, hinterher fahren.
Dies wird ermöglicht, indem der kleinste Abstandswert, welcher nicht null ist, aus dem Array welches der Laserscanner auf den Topic 'scan' veröffentlicht, herrausgesucht wird. ( Dann wird geschaut wie groß dieser Kleinste wert ist und ab einer bestimmten Distanz beginnt der Roboter sich immer in die richtung zu drehen wo der geringste Abstand ist.
Da das Array welches auf 'scan' gepublished wird 360 werte hat muss man die ersten 30 und die letzen 30 Werte nehmen um in einem 60 Grad winkel vor dem Roboter nach objekten zu schauen. (für jedes Grad einen Wert index 0 = Strahl in fahrtrichtung die werte sind im uhrzeigersinn aufsteigend)
Ist der geringste Abstandswert unter den ersten 30, dreht sich der roboter mit dem Uhrzeigersinn um den geringen Wert näher an 0 Grad herran zu bekommen. Indem er immer wieder nach links und rechts korrigiert und weiter fährt kann er so das objekt verfolge.
Ab einem bestimmten Abstand hält der Roboter jedoch auch einfach vor dem Objekt an.

Alle nötigen dependencies sind im Paket 'hier laserscann pakage namen einfügen' enthalten. Kopiere das Package in deinen ros2 WorkSpace und führe:
colcon build 
(oder colcon build --packages-select 'packagename' 'entrypoint/executable') 
und anschließen source install/setup.bash 
um das Program laufen zu lassen gib, nachdem du gebuildet hast in das Terminal ein:

ros2 run 'package name' 'executable' 

Der Roboter fährt nun gerade aus los und folgt dem nächsten Objekt was eine bestimmte Distanz unterschreitet, bis es vor ihm stehen bleibt.
Er beachtet nur die 60° die sich vor ihm befinden, also kann er stecken bleiben wenn man ihm zu nah an einer Ecke entlang leitet. 

## **Belegaufgabe 3 (Weiße Linie mit Kamera verfolgen)**
In der Dritten Belegaufgabe sollte der Roboter nun durch das auswerten des Kamerabildes, einer weißen Linie folgen. Außerdem soll das programm mit einer launchfile gestartet werden.
Um das Program nutzen zu können muss sich das Package 'linebounce' in ihrem ros2 Workspace befinden und zuerst mit 

colcon build (oder colcon build --packages-select linebounce)
gebuildet werden, dann muss die bash ausgeführt werden mit:
source install/setup.bash (oder . install/setup.bash)
anschließend kann die node mit 
ros2 launch linebounce linebounce_launch.py 
gestartet werden.

Der Roboter sollte auf der Linie starten, denn er Dreht sich, wenn er keine Linie erkennt erstmal nur auf der stelle und fährt nicht. 
Er erkennt die Linie indem die Unterste Reihe Pixel des Bildes in graustufen umgewandelt wird und man somit ein 640 Werte langes array welches die Helligkeit des Bodens genau vor dem Roboter wiederspiegelt. Wenn man nun überprüft wo die Höchste Konzentration von hellen Werten ist und den a Roboter anschließend rechts oder links drehen lässt, kann man sich nach der Linie ausrichten. (Aber natürlich nur wenn die Linie heller als der Rest des Bildes ist)
Durch entfernen der bildesränder kann beispielsweise noch etwas besser umgenagen werden, dass sich der Roboter gerne nach reflektionen oder anderen hellen dingen am Wegesrand ausrichten möchte. Für eine weiterführende verwendung sollten noch zusätzliche Logiken hinzugefügt werden, um der Straße verlässlicher und eventuell auch zwischendurch ohne linie, folgen zu können.

## **Belegaufgabe 4 (Zwischen 2 Objekten auf Linie fahren)**
Bei Aufgabe 4 soll der Roboter der Linie folgen, bis er ein objekt vor ihm nah genug mit dem Scanner erkennt und sich dann um 180° drehen und wieder der linie folgen. 
Das Programm kann verwendet werden indem das 'two_drive' sich in ihrem workspace befindet, und sie diesen befehl in der Workspace oder src directory ausführen :
colcon build --packages-select two_drive
danach:
. install/setup.bash
Nun können sie das Programm mit der launchfile starten. FÜhren sie den Befehl aus:

ros2 launch two_drive two_drive_launch.py

Anschließend sollte der Roboter anfangen der Linie zu folgen auf der er gestartet ist.

 Für diese aufgabe war wichtig, dass 3 Nodes gleichzeitig laufen, deshalb ist auch die launchfile so wichtig, da sie 3 Programme gleichzeitig startet. Eine Node ist für das folgen der Linie, eine für das Drehen an Objekten und eine für das Übermitteln der richtigen daten an den Roboter verantwortlich.
 
Die zwei ersten nodes laufen einfach vor sich hin und veröffentlichen jeder seine Fahrbefehele auf einem anderen topic, die 'MasterNode' kontrolliert dann, abhängig davon ob ein Objekt nah vor ihm ist, ob Fahrbefehle von der Linienfolgenden oder Drehenden Node an den Roboter weitergeleitet werden, so können nicht ausversehen mehrere Befehle auf einmal ankommen. 
