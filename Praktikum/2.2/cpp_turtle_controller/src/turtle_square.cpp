/*
Name:			turtle_square
Description:	Implementierung eines ROS 2 Nodes in C++, der Bewegungsbefehle
				an das Topic /turtle1/cmd_vel sendet, um die Turtle in der 
				turtlesim-Simulation ein Quadrat fahren zu lassen.
				Die Steuerung erfolgt zeitbasiert (Open-Loop):
				- Es werden Twist-Nachrichten mit linearer und angularer Geschwindigkeit gesendet
				- Eine definierte Strecke wird gefahren, dann 90° gedreht
				- Zyklus wird kontinuierlich wiederholt
*/

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ Client-Bibliothek
#include "geometry_msgs/msg/twist.hpp" // Nachrichtentyp für Bewegungsbefehle
#include <chrono> // Für Zeitfunktionen
using namespace std::chrono_literals; // Ermöglicht 100ms-Schreibweise

// Hauptklasse der Node
class TurtleController : public rclcpp::Node {
public:
	TurtleController() : Node("turtle_controller") {
		// Publisher initialisieren: sendet Twist-Nachrichten an /cmd_vel
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

		// Timer erzeugen, der alle 100 Millisekunden move() aufruft
		timer_ = this->create_wall_timer(100ms, std::bind(&TurtleController::move, this));

		RCLCPP_INFO(this->get_logger(), "TurtleController Node gestartet");
	}

private:
	// Bewegungsfunktion, wird periodisch aufgerufen
	void move() {
		geometry_msgs::msg::Twist msg; // Neue Nachricht vom Typ Twist

		// Steuerlogik:
		// Schrittweise Fahrt vorwärts, dann Drehung um 90°
		// Schritt = 0.1 s (Timer)
		if (step_ < 20) { 
			msg.linear.x = 1.0; // Vorwärtsgeschwindigkeit 
			msg.angular.z = 0.0;// Keine Drehung
		} else if (step_ < 36) { // 16 Steps × 0.1 s = 1.6 s Drehung
			msg.linear.x = 0.0; // Anhalten
			msg.angular.z = 1.0; // Drehgeschwindigkeit (≈ 90° über 1.6 s)
		} else {
			step_ = 0; // Zyklus zurücksetzen
		}

		// Nachricht veröffentlichen
		publisher_->publish(msg);

		// Konsolenausgabe der Bewegung
		RCLCPP_INFO(this->get_logger(), "Schritt: %d | linear: %.2f | angular: %.2f", 
			step_, msg.linear.x, msg.angular.z);

		step_++; // Schrittzähler erhöhen
	}

	// Private Membervariablen
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // Publisher-Objekt
	rclcpp::TimerBase::SharedPtr timer_; // Timer für move()-Aufrufe
	int step_ = 0; // interner Schrittzähler
};

// main()-Funktion: Einstiegspunkt der Node
int main(int argc, char *argv[]) {
	// Initialisierung des ROS 2-Systems
	rclcpp::init(argc, argv);

	// Node starten
	rclcpp::spin(std::make_shared<TurtleController>());

	// Nach Beendigung aufräumen
	rclcpp::shutdown();
	return 0;
}
