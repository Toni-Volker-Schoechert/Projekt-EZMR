/*
Name:	bumper_react
Description:
	Implementierung eines ROS 2 Nodes in C++, der Bewegungsbefehle
	an das Topic /cmd_vel sendet, um den Roboter automatisch fahren zu lassen.
	Die Steuerung erfolgt ereignisgesteuert über die Bumper-Sensoren.

Funktionsweise:
	- Standardmäßig fährt der Roboter geradeaus.
	- Wird der linke Bumper gedrückt:
		Buzzer sendet Hohen ton
		kurz zurückfahren, dann nach rechts drehen, dann wieder geradeaus.
	- Wird der rechte (oder beide) Bumper gedrückt:
		Buzzer sendet Tiefen ton
		kurz zurückfahren, dann nach links drehen, dann wieder geradeaus.
	(beim rückwärtsfahren beept der mehrfach)

Ziel:
	Dieses Verhalten simuliert eine einfache Hindernisvermeidung.
*/

#include "rclcpp/rclcpp.hpp"// ROS 2 C++ API
#include "std_msgs/msg/bool.hpp"// Für die Bumper-Nachrichten
#include "geometry_msgs/msg/twist.hpp"// Für Bewegungsbefehle (cmd_vel)
#include "std_msgs/msg/int32.hpp"// für buzzer Befehle
#include <chrono>// Für Zeitverzögerungen
using namespace std::chrono_literals;

// Hauptklasse für den Node
class BumperReactNode : public rclcpp::Node
{
public:
	BumperReactNode() : Node("bumper_react_node") {
		// Publisher für Bewegungsbefehle (Twist)
		cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		
		// Publisher für Buzzer Befehle
		buzzer_pub_ = this->create_publisher<std_msgs::msg::Int32>("/buzzer", 10);

		// Subscriber für die beiden Bumper-Sensoren
		left_bumper_sub_ = this->create_subscription<std_msgs::msg::Bool>(
			"/bumper/left", 10,
			std::bind(&BumperReactNode::left_bumper_callback, this, std::placeholders::_1));

		right_bumper_sub_ = this->create_subscription<std_msgs::msg::Bool>(
			"/bumper/right", 10,
			std::bind(&BumperReactNode::right_bumper_callback, this, std::placeholders::_1));

		// Timer für reguläre Bewegungsaktualisierung alle 100 ms
		timer_ = this->create_wall_timer(200ms, std::bind(&BumperReactNode::update, this));
		
		send_stop();
		RCLCPP_INFO(this->get_logger(), "BumperReactNode gestartet – fährt geradeaus, bis Bumper ausgelöst werden.");
	}
	
	~BumperReactNode() { 
		send_stop(); 
		RCLCPP_INFO(get_logger(), "Node beendet – Stop gesendet.");
	}

private:
	// Zustände der Bewegungslogik
	enum class State { FORWARD, REVERSE, STOP, TURN };

	// Aktueller Zustand
	State state_ = State::FORWARD;

	// Speichert die gewünschte Drehrichtung ("left" oder "right")
	std::string turn_dir_ = "none";

	// Zählt verbleibende Timer-Ticks für die aktuelle Bewegung
	int counter_ = 0;

	// Wird true, wenn gerade eine Reaktion läuft (verhindert Überschneidungen)
	bool reacting_ = false;
	
	//Hilfsfunktionen
	// Stopp Befehl senden
	void send_stop(){
		geometry_msgs::msg::Twist stop;
		stop.linear.x = 0.0;
		stop.angular.z = 0.0;
		cmd_pub_->publish(stop);
	}
	
	// Bewegungsbefehl senden
	void send_twist(float lin, float ang) {
		geometry_msgs::msg::Twist cmd;
		cmd.linear.x = lin;
		cmd.angular.z = ang;
		cmd_pub_->publish(cmd);
	}
	
	// Buzzer Befehl senden
	void send_buzzer(int freq) {
		std_msgs::msg::Int32 msg;
		msg.data = freq;
		buzzer_pub_->publish(msg);
	}


	// Callback-Funktionen für Bumper-Ereignisse

	// linke Bumper gedrückt
	void left_bumper_callback(const std_msgs::msg::Bool::SharedPtr msg) {
		if (msg->data && !reacting_) { // nur reagieren, wenn gerade keine Aktion läuft
			RCLCPP_WARN(get_logger(), "Linker Bumper gedrückt!");
			send_buzzer(200); // höher
			start_reaction("left"); // Reaktion einleiten
		}
	}

	// rechte Bumper gedrückt
	void right_bumper_callback(const std_msgs::msg::Bool::SharedPtr msg) {
		if (msg->data && !reacting_) {
			RCLCPP_WARN(get_logger(), "Rechter Bumper gedrückt!");
			send_buzzer(50); // Tief
			start_reaction("right"); // Reaktion einleiten
		}
	}

	// Reaktionslogik

	// Startet Bumper-Reaktion (rückwärts → stopp → drehen → vorwärts)
	void start_reaction(const std::string &side) {
		reacting_ = true;	// blockiert neue Reaktionen während der laufenden
		state_ = State::REVERSE;// Beginne mit Rückwärtsfahren
		counter_ = 10;		// 10 × 100 ms = 1 Sekunde Rückwärtsfahrt
		turn_dir_ = side;	// Richtung merken für Drehen 

		RCLCPP_INFO(get_logger(), "Starte Reaktion: Rückwärts + Drehung (%s)", side.c_str());
	}

	// Bewegungssteuerung (wird vom Timer aufgerufen)
	void update() {
		geometry_msgs::msg::Twist cmd; // Nachricht, die an /cmd_vel gesendet wird

		switch (state_)
		{
		case State::FORWARD:
			// Normalbetrieb: fahre geradeaus
			if (!reacting_) {
				send_twist(1.0, 0.0); // normal vorwärts
			}
			break;

		case State::REVERSE:
			// Rückwärtsfahrt für begrenzte Zeit
			send_twist(-5.0, 0.0); // rückwärts
			if (counter_ % 2 ==1) send_buzzer(2000);  // sendet jeden 2. Tick (alle 400 ms)
			
			if (--counter_ <= 0) {
				state_ = State::STOP; // nächster Zustand: Stopp
				counter_ = 10;         
				RCLCPP_INFO(get_logger(), "Rückwärtsfahrt beendet, Stopp-Phase.");
			}
			break;

		case State::STOP:
			// Roboter steht still
			send_stop();
			if (--counter_ <= 0) {
				state_ = State::TURN; // nächster Zustand: Drehung
				counter_ = 10;        
				RCLCPP_INFO(get_logger(), "Starte Drehung (%s).", turn_dir_.c_str());
			}
			break;

		case State::TURN:
			// Drehbewegung nach links oder rechts
			if (turn_dir_ == "left") send_twist(0.0, -4.0);
			else send_twist(0.0, 4.0);
			if (--counter_ <= 0) {
				state_ = State::FORWARD;
				reacting_ = false;
				send_stop();
				RCLCPP_INFO(get_logger(), "Drehung beendet, fahre weiter geradeaus.");
			}
			break;
		}
	}

	// Membervariablen
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;   // Publisher /cmd_vel
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr buzzer_pub_; // Publisher buzzer
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_bumper_sub_;  // linker Bumper
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_bumper_sub_; // rechter Bumper
	rclcpp::TimerBase::SharedPtr timer_; // Timer für zyklische Updates
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv); // ROS 2 initialisieren
	rclcpp::spin(std::make_shared<BumperReactNode>()); // Node starten (läuft bis Ctrl+C)
	rclcpp::shutdown(); // ROS 2 sauber beenden
	return 0;
}

