#include <EPB_Encoder.h>
#include <EPB_Encoder2.h>
#include <EPB_DCmotor.h>
#include <Servo.h>



//defini les numéro de pin pour le contrôle des moteurs
#define direction_droite 2
#define PWM_droite 3
#define direction_gauche 4
#define PWM_gauche 5
//defini les numéro de pin pour les encodeurs
#define ENC_CHA_droite 9
#define ENC_CHB_droite 11
#define ENC_CHA_gauche 10
#define ENC_CHB_gauche 12

//parametre du servo moteur
Servo servo;
#define servoPIN 6
const int angle = 90;
//defini des classe pour les moteurs
EPB_DCmotor motor1(PWM_gauche, direction_gauche);
EPB_Encoder encoder1(ENC_CHA_gauche, ENC_CHB_gauche);
EPB_DCmotor motor2(PWM_droite, direction_droite);
EPB_Encoder2 encoder2(ENC_CHA_droite, ENC_CHB_droite);

//defini PI
#define PI 3.1415926535897932384626433832795

//defini les paramètres de notre système
const float RAYON_COURROIE = 0.00575; //rayon de la couroie utilisée
const float A = 1.33; // Largeur du tableau
const float B  = 1.20;   // Hauteur du tableau
const float VMAX = 60;   // defini la vitesse maximal(en tension) à laquelle peuvent tourner nos moteurs
//paramètres du cercle
const float R = 0.15; // Rayon du cercle à tracer
#define nombre_de_points_cercle 150
const float OMEGA = (2*PI)/nombre_de_points_cercle; // angle par points


//paramètres du carré
const float longeur_cotes = 0.15;
const float C_H = (B/2 - longeur_cotes/2);//hauteur du coté haut
const float C_B = (B/2 + longeur_cotes/2);//hauteur du coté bas
const float C_G = (A/2 - longeur_cotes/2);//abscisse du coté gauche
const float C_D = (A/2 + longeur_cotes/2);//abscisse du coté droite

#define nbr_points_cotes 70
float LAMBDA = longeur_cotes/nbr_points_cotes;//longeur entre 2 points

//variable utilisé pour la communication avec le programme python
int corde1 = 0;
int corde2 = 0;
int pen = 0;
String entree;
int lecture;

//introduit une structure de type (x,y), un vecteur de dimension 2 utilisé par la suite pour caractérisé les positions
struct Vecteur{
  float g;
  float d;
  float p;
}typedef Vecteur;

// defini les parametres du regulateur P
const double p = 1;

//variables des vitesses et poisitions des moteurs utilisées par le regulateur P
Vecteur vitesse_reel, positions, old_positions;
Vecteur destination;

//variables des positions
float position_initial1 = (2*360*sqrt(sq((A/2)-0.08) + sq((B/2)-0.09)))/(RAYON_COURROIE*2*PI);//nombres de ticks des cordes
float position_initial2 = (2*360*sqrt(sq(A-(A/2+0.08)) + sq((B/2)-0.09)))/(RAYON_COURROIE*2*PI);


// varaible de texte permettant de commander l'arduino
char c;
char coordonnee_python;
long nextTime1;
int state;// variable pour commencer les formes


void setup() {
  Serial.begin(9600);  //commence la communication avec l'ordinateur grace aux ports séries
  motor1.begin(); //initie le pilotage des moteurs
  motor2.begin(); //initie le pilotage des moteurs
  encoder1.begin(); //initie la lecture des encodeurs
  encoder2.begin2(); //initie la lecture des encodeurs
  servo.attach(servoPIN); // initie le contrôle du servo-moteur

  Serial.setTimeout(0.5); // definit de le temps maximal d'attente des fonction lisants les ports série a 0.5 s
  
  positions.g = lire_position1(encoder1); //mets a jour les positions des moteurs
  positions.d = lire_position2(encoder2); //mets a jour les positions des moteurs
  destination.g = position_initial1; //mets a jour la première destination des moteurs
  destination.d = position_initial2; //mets a jour la première destination des moteurs
  
  

  nextTime1 = millis();
  state = 0;
}

void loop() {
  
  deplacement_manuel(); //cette fonction est appelé en premier pour pouvoir replacer le prototype à la position voulues
  //cette partie permet de tester les encodeurs en observant les ports séries en tournant les moteurs à la main
  /*positions.g = lire_position1(encoder1);//mets a jour les positions des moteurs
  positions.d = lire_position2(encoder2);//mets a jour les positions des moteurs
  Serial.print(positions.g);
  Serial.print(",");
  Serial.println(positions.d);*/
  
  //cette condition permet de de ne pas commencer le code tout de suite mais d'attendre l'envoie d'un signal ("B") sur les ports série de la part de l'utilisateur
  if (Serial.available()){
    c = Serial.read();
    if(c == 'B'){
      state = 1;
    }
  }

  if(state == 1){//effectuer ici les fonctions voulues

    //tracer_carre();
    tracer_cercle();
    //test_encodeur();
    //tracer_depuis_python();

    state = 0;
  }
}
