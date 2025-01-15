float sign(float x){
  //fonction retournant le signe d'un float
if (x<0){
  return -1;
}else{
  return 1;
}
}
void deplacement_et_regulation(Vecteur destinations){

//fonction permettant de se déplacer entre 2 points en applicant une régulation P.
//elle contrôle aussi le levage de bic

  float e_g,e_d,tension_g,tension_d;//erreur gauche, erreur droite, tension gauche, tension droite
  int interval = 10;
  float previous_millis = 0;
  positions.g = lire_position1(encoder1);//mets a jour les positions des moteurs
  positions.d = lire_position2(encoder2);//mets a jour les positions des moteurs

  pen_management(destinations); // lève eou baisse le bic si nécessaire en fonction de l'instrcution se trouvant dasn le vecteur destinations

  while(sqrt(sq(destinations.g - positions.g)) >=30 || sqrt(sq(destinations.d - positions.d)) >=30){
    float current_millis = millis();
    if(current_millis - previous_millis >= interval){
            e_g = destinations.g - positions.g;//calcul les erreurs
            e_d = 1.2*( destinations.d - positions.d);//calcul les erreurs
            tension_g = e_g*p;//applique le facteur p de la regulation
            tension_d = e_d*p;//applique le facteur p de la regulation
            tension_g = constrain(tension_g, -VMAX, VMAX);
            tension_d = constrain(tension_d, -VMAX, VMAX);

            motor1.setSpeed(tension_g - 15);//applique la tension aux moteurs (à vérifier)
            motor2.setSpeed(tension_d - 15);//applique la tension aux moteurs

            positions.g = lire_position1(encoder1);//mets a jour les positions des moteurs
            positions.d = lire_position2(encoder2);//mets a jour les positions des moteurs
            previous_millis = current_millis; //met a jour le temps


            /*Serial.print("erreur:");
            Serial.print(e_g);
            Serial.print(",");
            Serial.print(e_d);
            Serial.print("    tensions:");
            Serial.print(tension_g);
            Serial.print(",");
            Serial.println(tension_d);*/
    }
            /*Serial.print(previous_millis);
            Serial.print(",");
            Serial.println(current_millis);*/
            
  }
}
Vecteur coordonnees_carre(float t){

//fonction générant des coordonnées d'un carré en fonction d'un paramètre t (le temps)

  float r1,r2,x,y;
  if(t<=nbr_points_cotes){
    x = C_G+LAMBDA*t;
    y = C_H;
    r1 = sqrt(sq(x-0.08)+sq(y-0.09));
    r2 = sqrt(sq(A-(x+0.08))+sq(y-0.09));
  }
  else if(t>nbr_points_cotes && t<=2*nbr_points_cotes){
    x = C_D;
    y = C_H+LAMBDA*(t-nbr_points_cotes);
    r1 = sqrt(sq(x-0.08)+sq(y-0.09));
    r2 = sqrt(sq(A-(x+0.08))+sq(y-0.09));
  }
  else if(t>2*nbr_points_cotes && t<=3*nbr_points_cotes){
    x = C_D-LAMBDA*(t-2*nbr_points_cotes);
    y = C_B;
    r1 = sqrt(sq(x-0.08)+sq(y-0.09));
    r2 = sqrt(sq(A-(x+0.08))+sq(y-0.09));
  }
  else if(t>3*nbr_points_cotes && t<=4*nbr_points_cotes){
    x = C_G;
    y = C_B-LAMBDA*(t-3*nbr_points_cotes);
    r1 = sqrt(sq(x-0.08)+sq(y-0.09));
    r2 = sqrt(sq(A-(x+0.08))+sq(y-0.09));
  }
    r1 = sqrt(sq(x-0.08)+sq(y-0.09));
    r2 = sqrt(sq(A-(x+0.08))+sq(y-0.09));
  Vecteur rayon;
  rayon.g = (2*360*r1)/(2*PI*RAYON_COURROIE);// en ° -> en positions
  rayon.d = (2*360*r2)/(2*PI*RAYON_COURROIE);// en ° -> en positions
  return rayon;

}
Vecteur coordonnees_cercle(float t){

//fonction générant des coordonnées d'un cercle en fonction d'un paramètre t (le temps)

  float x = (A/2 + R*cos(OMEGA*t));
  float y = (((3*B)/4) - R*sin(OMEGA*t)); 
  float r1 = sqrt(sq(x-0.08)+sq(y-0.09));
  float r2 = sqrt(sq(A-(x+0.08))+sq(y-0.09));

  Vecteur rayon;
  rayon.g = (2*360*r1)/(2*PI*RAYON_COURROIE);// en ° -> en positions
  rayon.d = (2*360*r2)/(2*PI*RAYON_COURROIE);// en ° -> en positions
  return rayon;
}
float lire_position1(EPB_Encoder encoder1){

//fonction qui lit la position du moteur 1

      positions.g = position_initial1 - encoder1.read();
      return positions.g;


}
float lire_position2(EPB_Encoder2 encoder2){

//fonction qui lit la position du moteur 2

      positions.d = position_initial2 + encoder2.read2();
      return positions.d;
}
void lire_vitesse1(){

//fonction qui lit la vitesse du moteur 1

        lire_position1(encoder1);
        vitesse_reel.g = positions.g - old_positions.g;
        old_positions.g = positions.g;

}
void lire_vitesse2(){

//fonction qui lit la vitesse du moteur 2

        lire_position2(encoder2);
        vitesse_reel.d = positions.d - old_positions.d;
        old_positions.d = positions.d;
}
void deplacement_manuel(){

/*fonction permettant de deplacer le robot manuellement 
  A -> moteur gauche vers le haut
  Q -> moteur gauche vers le bas
  Z -> moteur droit vers le haut
  S -> moteur droit vers le bas
  E -> arrete les moteurs

il faut envoyer ces lettres sur les ports série pour controller le robot.*/

  if (Serial.available()) {
    c = Serial.read();
    if (c == 'A'){
      motor1.setSpeed(-100);
    }
    else if(c == 'Q'){
      motor1.setSpeed(100);
    }
    else if(c == 'Z'){
      motor2.setSpeed(-100);
    }
    else if(c == 'S'){
      motor2.setSpeed(100);
    }
    else if(c == 'E'){
      motor2.setSpeed(0);
      motor1.setSpeed(0);
    }
    else if(c == 'U'){
      pen_up();
    }
    else if(c == 'L'){
      pen_down();
    }
  }

}
void tracer_cercle(){

// fonction qui trace un cerlce en génrant des coordonnées a partir d'un variable t 
//et en faisant déplacer le robot grace a la fonction déplacement_et_regulation


    pen_down();
    destination = coordonnees_cercle(0);
    destination.p = 1;
    deplacement_et_regulation(destination);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    pen_up();


    float t;
          for(t=1; t <= nombre_de_points_cercle; t+=1){
          destination = coordonnees_cercle(t);
          destination.p = 0;
          deplacement_et_regulation(destination);
          /*Serial.print(positions.g);
          Serial.print(",");
          Serial.print(positions.d);
          Serial.print(",");
          Serial.print(destination.g);
          Serial.print(",");
          Serial.println(destination.d);*/
          }
          motor1.setSpeed(0);
          motor2.setSpeed(0);
          pen_down();
          //delay(2000);
          destination.g = position_initial1;
          destination.d = position_initial2;
          destination.p = 1;
          deplacement_et_regulation(destination);
          motor1.setSpeed(0);
          motor2.setSpeed(0);
}
void tracer_carre(){

// fonction qui trace un cerlce en génrant des coordonnées a partir d'un variable t 
//et en faisant déplacer le robot grace a la fonction déplacement_et_regulation

    /*
    //pen_up();
    //delay(100);
    destination = coordonnees_carre(0);
    deplacement_et_regulation(destination);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    //pen_down();
    //delay(2000);*/
    float t;
          for(t=1; t <= 4*nbr_points_cotes; t+=1){
            /*Serial.print(positions.g);
            Serial.print(",");
            Serial.print(positions.d);
            Serial.print(",");
            Serial.print(destination.g);
            Serial.print(",");
            Serial.println(destination.d);*/ 
            destination = coordonnees_carre(t);
            deplacement_et_regulation(destination);
          motor1.setSpeed(0);
          motor2.setSpeed(0);
          }
          motor1.setSpeed(0);
          motor2.setSpeed(0);
          //pen_up();
          //delay(2000);
          destination.g = position_initial1;
          destination.d = position_initial2;
          deplacement_et_regulation(destination);
          motor1.setSpeed(0);
          motor2.setSpeed(0);

}
void tracer_depuis_python(){
int boucle = 0;
/* fonction qui permet de tracer n'importe quel image ou text.
  Pour cela, on recoit des coordonnées générées dans un logiciel de dessin vectioriel 
  et envoyées sur les ports série grace à un programme python.

  Ces coordonnées sont envoyé une à une et sont ensuite lue par le programme arduino 
  qui enregistre ces coordonnées dans le vecteur destinations et déplace la robot à la bonne coordonée.

  une fois la destination atteinte le programme arduino envoie un signal ("k") au programme python 
  pour lui signaler qu'il peut envoyer la prochaines coordonnée.

  Ces coordonées contiennent la postion à atteindre mais aussi si le robot doit l'atteindre avec le bic levé ou baissé */

  Serial.print("lancez le programme python");
  while(true){

  positions.g = lire_position1(encoder1);//mets a jour les positions des moteurs
  positions.d = lire_position2(encoder2);//mets a jour les positions des moteurs
  Vecteur new_destination = positions; //mets a jours les variables de destination pour qu'elle se trouve initialement au la positions du robot
  destination = positions;
  /*Serial.print(positions.g);
  Serial.print(",");
  Serial.print(positions.d);
  Serial.print(",");
  Serial.print(destination.g);
  Serial.print(",");
  Serial.print(destination.d);
  Serial.print(new_destination.g);
  Serial.print(",");
  Serial.println(new_destination.d);*/
  
  // ajoute une condtion au déplacement, il faut que la coordonnée voules soit différente de celle actuelle
  /*while(sqrt(sq(new_destination.g - destination.g)) < 15 && sqrt(sq(new_destination.d - destination.d)) < 15){
    Serial.println("JE LIS");
    new_destination = lecture_signaux();
    if(sqrt(sq(new_destination.g - destination.g)) < 15 && sqrt(sq(new_destination.d - destination.d)) < 15){
      Serial.println("k");
    }
  }*/
  /*while(new_destination.g == destination.g && new_destination.d == destination.d){
   //Serial.print("JE LIS");
   new_destination = lecture_signaux();
    if(new_destination.g == destination.g && new_destination.d == destination.d){
     Serial.println("k");
    }
   }*/
   new_destination = lecture_signaux();
   destination = new_destination;
   deplacement_et_regulation(destination);

   motor1.setSpeed(0);
   motor2.setSpeed(0);
   delaie(10);
   Serial.println("k");
   if (Serial.available()){
   char m = Serial.peek();
    if(m == "f"){
      Serial.println("v");
      boucle = 1;
    }
   }
  } 
          
          destination.g = position_initial1;
          destination.d = position_initial2;
          destination.p = 0;
          deplacement_et_regulation(destination);
          motor1.setSpeed(0);
          motor2.setSpeed(0);
}
void pen_up(){
  delaie(100);
  for(int val = 0; val <= 90; val ++){
  servo.write(val);                  // sets the servo position according to the scaled value
  delaie(20);  // waits for the servo to get there
  }
  delaie(100);
  

}
void pen_down(){
  
  delaie(100);
  for(int val = 90; val >= 0; val --){
  servo.write(val);                  // sets the servo position according to the scaled value
  delaie(20);  // waits for the servo to get there
  }
  delaie(100);
}
void pen_management(Vecteur destination){
  if(sqrt(sq(angle - servo.read())) <= 30){
    if(destination.p == 1){
      pen_down();
    }
  }else{
    if(destination.p == 0){
      pen_up();
    }
  }
}
void delaie(int x){
  long temps = millis();
  while(millis() <= temps + x){
  }
}
Vecteur lecture_signaux(){
  long int chiffre;
  String digit;
  Vecteur cordes;
  int etat = 0;
  String activation;
  while (etat == 0){
    if(Serial.available() > 0){ 
    activation = Serial.readString();
    if(String(activation) == "O"){
      etat = 1;
      }
    }
  }  
   while(!Serial.available());
   delaie(20);
   entree = Serial.readStringUntil('\n');
   
   //Serial.print(entree);
   for(int i = 0; i<= entree.length()-1; i+=1){
    if(isdigit(entree[i]) && lecture == 0){      
      digit = entree[i];
      chiffre = digit.toInt();
      corde1 *= 10;
      corde1 += chiffre;
    }else if( !(isdigit(entree[i])) && lecture == 0){
      lecture += 1;
    }else if(isdigit(entree[i]) && lecture == 1){
      digit = entree[i];
      chiffre = digit.toInt();
      corde2 *= 10;
      corde2 += chiffre;
    }else if(!(isdigit(entree[i])) && lecture == 1){
      lecture +=1;
    }else if(isdigit(entree[i]) && lecture == 2){
      digit = entree[i];
      chiffre = digit.toInt();
      pen = chiffre;
      //Serial.print(corde1);
      cordes.g=corde1;
      cordes.d=corde2;
      cordes.p=pen;
      
      corde1 = 0;
      corde2 = 0;
      lecture = 0;
      
      
      /*Serial.print(cordes.g);
      Serial.print(",");
      Serial.print(cordes.d);
      Serial.print(",");
      Serial.print(cordes.p);*/
      return(cordes);
    }
   }
}
void test_encodeur(){
  
  positions.g = lire_position1(encoder1);//mets a jour les positions des moteurs
  positions.d = lire_position2(encoder2);//mets a jour les positions des moteurs
  destination.g = positions.g - 360*7;
  destination.d = positions.d - 360*7;
  deplacement_et_regulation(destination);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  
}

class vecteur(){
  float g;
  
}