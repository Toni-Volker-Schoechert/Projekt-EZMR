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
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp" //für hazard detection was bumper signale enthält
#include "geometry_msgs/msg/twist.hpp"// Für Bewegungsbefehle (cmd_vel)
#include "irobot_create_msgs/msg/audio_note_vector.hpp"//für buzzer befehle
#include "irobot_create_msgs/msg/audio_note.hpp"//für buzzer befehle
#include <chrono>// Für Zeitverzögerungen

using namespace std::chrono_literals;
using irobot_create_msgs::msg::AudioNoteVector;
using irobot_create_msgs::msg::AudioNote;

// Hauptklasse für den Node
class BumperReactNode : public rclcpp::Node
{
public:
	BumperReactNode() : Node("bumper_react_node") {
		// Publisher für Bewegungsbefehle (Twist)
		cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		
		// Publisher für Buzzer Befehle
		buzzer_pub_ = this->create_publisher<AudioNoteVector>("/cmd_audio", 10);

		// Subscriber für unteranderem die beiden Bumper-Sensoren
		auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

		hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
    			"/hazard_detection", qos,
    			std::bind(&BumperReactNode::hazard_callback, this, std::placeholders::_1));

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
		AudioNoteVector notes;
		AudioNote note;
		note.frequency = freq;
		note.max_runtime.sec = 0;
		note.max_runtime.nanosec = 500000000;  // 0.5 Sekunden
		notes.notes.push_back(note);
		buzzer_pub_->publish(notes);
	}


	// Callback-Funktionen für Bumper-Ereignisse

	void hazard_callback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg) {
		if (reacting_) return;  // blockiert neue Aktionen während einer laufenden Reaktion

		for (auto &detection : msg->detections) {
			if (detection.type == 1) {
				std::string frame = detection.header.frame_id;
				RCLCPP_WARN(this->get_logger(), "Bumper ausgelöst: %s", frame.c_str());
				//linke Bumper
            			if (frame.find("left") != std::string::npos) {
                			send_buzzer(200);  // hoher Ton
                			start_reaction("left");
            			}
            			else if (frame.find("right") != std::string::npos) {
                			send_buzzer(800);   // tiefer Ton
					start_reaction("right");
            			}
				else {
					send_buzzer(100);  // mittlerer Ton bei mittlerem Bumper
					start_reaction("right");  // standardmäßig rechts drehen
				}
            			break;  // Nur auf ersten Treffer reagieren
			}
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
				send_twist(0.15, 0.0); // normal vorwärts
			}
			break;

		case State::REVERSE:
			// Rückwärtsfahrt für begrenzte Zeit
			send_twist(-0.10, 0.0); // rückwärts
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
			if (turn_dir_ == "left") send_twist(0.0, -1.0);
			else send_twist(0.0, 1.0);
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
	rclcpp::Publisher<irobot_create_msgs::msg::AudioNoteVector>::SharedPtr buzzer_pub_; // Publisher buzzer
	rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_; //enthält bumper
	rclcpp::TimerBase::SharedPtr timer_; // Timer für zyklische Updates
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv); // ROS 2 initialisieren
	rclcpp::spin(std::make_shared<BumperReactNode>()); // Node starten (läuft bis Ctrl+C)
	rclcpp::shutdown(); // ROS 2 sauber beenden
	return 0;
}
