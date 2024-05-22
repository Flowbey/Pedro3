bool walk(int repeat , int x, int y, int z, float yaw ){
  /*
  x = 0;
  y = 20;
  z = 20;
  */
  paul.setMoveTime(1,servoTime);


  if(introFlag==false){
    paul.setCoordinatesRelative(front_left,   0,    0,   0, 0);
    paul.setCoordinatesRelative(middle_left,  0,    0,   -z, 0);
    paul.setCoordinatesRelative(rear_left,    0,    0,   0, 0);
    paul.setCoordinatesRelative(front_right,  0,    0,   -z, 0);
    paul.setCoordinatesRelative(middle_right, 0,    0,   0, 0);
    paul.setCoordinatesRelative(rear_right,   0,    0,   -z, 0);
    paul.moveLegsAcclerate();
    introFlag=true;
  }

  paul.setMoveTime(1,servoTime/4);
  paul.setCoordinatesRelative(front_left,   -x/2,    -y/2,   0, -yaw/2);
  paul.setCoordinatesRelative(middle_left,  x/2,    y/2,   0, yaw/2);
  paul.setCoordinatesRelative(rear_left,    -x/2,    -y/2,   0, -yaw/2);
  paul.setCoordinatesRelative(front_right,  x/2,    y/2,   0, yaw/2);
  paul.setCoordinatesRelative(middle_right, -x/2,    -y/2,   0, -yaw/2);
  paul.setCoordinatesRelative(rear_right,   x/2,    y/2,   0, yaw/2);
  paul.moveLegsAcclerate();


  paul.setMoveTime(1,servoTime);
  paul.setCoordinatesRelative(front_left,   0,    0,   0, 0);
  paul.setCoordinatesRelative(middle_left,  0,    0,   z, 0);
  paul.setCoordinatesRelative(rear_left,    0,    0,   0, 0);
  paul.setCoordinatesRelative(front_right,  0,    0,   z, 0);
  paul.setCoordinatesRelative(middle_right, 0,    0,   0, 0);
  paul.setCoordinatesRelative(rear_right,   0,    0,   z, 0);
  paul.moveLegsAcclerate();


  paul.setCoordinatesRelative(front_left,   0,    0,   -z, 0);
  paul.setCoordinatesRelative(middle_left,  0,    0,   0, 0);
  paul.setCoordinatesRelative(rear_left,    0,    0,   -z, 0);
  paul.setCoordinatesRelative(front_right,  0,    0,   0, 0);
  paul.setCoordinatesRelative(middle_right, 0,    0,   -z, 0);
  paul.setCoordinatesRelative(rear_right,   0,    0,   0, 0);
  paul.moveLegsAcclerate();


  paul.setMoveTime(1,servoTime/2);
  paul.setCoordinatesRelative(front_left,   x,    y,   0, yaw);
  paul.setCoordinatesRelative(middle_left,  -x,    -y,   0, -yaw);
  paul.setCoordinatesRelative(rear_left,    x,    y,   0, yaw);
  paul.setCoordinatesRelative(front_right,  -x,    -y,   0, -yaw);
  paul.setCoordinatesRelative(middle_right, x,    y,   0, yaw);
  paul.setCoordinatesRelative(rear_right,   -x,    -y,   0, -yaw);
  paul.moveLegsAcclerate();


  paul.setMoveTime(1,servoTime);
  paul.setCoordinatesRelative(front_left,   0,    0,   z, 0);
  paul.setCoordinatesRelative(middle_left,  0,    0,   0, 0);
  paul.setCoordinatesRelative(rear_left,    0,    0,   z, 0);
  paul.setCoordinatesRelative(front_right,  0,    0,   0, 0);
  paul.setCoordinatesRelative(middle_right, 0,    0,   z, 0);
  paul.setCoordinatesRelative(rear_right,   0,    0,   0, 0);
  paul.moveLegsAcclerate();


  paul.setCoordinatesRelative(front_left,   0,    0,   0, 0);
  paul.setCoordinatesRelative(middle_left,  0,    0,   -z, 0);
  paul.setCoordinatesRelative(rear_left,    0,    0,   0, 0);
  paul.setCoordinatesRelative(front_right,  0,    0,   -z, 0);
  paul.setCoordinatesRelative(middle_right, 0,    0,   0, 0);
  paul.setCoordinatesRelative(rear_right,   0,    0,   -z, 0);
  paul.moveLegsAcclerate();


  paul.setMoveTime(1,servoTime/4);
  paul.setCoordinatesRelative(front_left,   -x/2,    -y/2,   0, -yaw/2);
  paul.setCoordinatesRelative(middle_left,  x/2,    y/2,   0, yaw/2);
  paul.setCoordinatesRelative(rear_left,    -x/2,    -y/2,   0, -yaw/2);
  paul.setCoordinatesRelative(front_right,  x/2,    y/2,   0, yaw/2);
  paul.setCoordinatesRelative(middle_right, -x/2,    -y/2,   0, -yaw/2);
  paul.setCoordinatesRelative(rear_right,   x/2,    y/2,   0, yaw/2);
  paul.moveLegsAcclerate();
  paul.setMoveTime(1,servoTime);

  return 1;
}
