Ce code est utilisé dans une modélisation de train à sustentation magnétique où le but est de le maintenir à une hauteur de consigne ( 0.75mm ).
Ici, le capteur VL53L4CD (Datasheet : voir document "VL53L4CD.pdf" ) permet d'acquérir la distance.
Cette acquisition est ensuite filtrée par des paramètres que l'on peut faire varier pour plus ou moins de rapidité.
À travers les ports TX et RX, la carte Méga transmet les données à une carte UNO qui pilote la bobine en intensité. 
