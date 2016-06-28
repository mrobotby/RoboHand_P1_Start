/*
    РОБОТЫ и НАБОРЫ ПО РОБОТОТЕХНИКЕ на МРобот! mrobot.by
    http://www.mrobot.by

    Кухня Роботов <maxxlife>
    http://www.vk.com/cookrobot
    Copyright (c) Кухня роботов <maxxlife.by> All rights reserved.
    Copyright (c) Макслайф Робот Maxxlife Robot All rights reserved.
    Наши лаборатории по робототехнике:
    Ленинская РОС ДОСААФ, ул. Рокоссовского 63/2, Минск, Беларусь
    Подробнее в нашей группе Вконтакте http://www.vk.com/cookrobot
    И на сайте http://www.maxxlife.by
    ****************************************************
    Мы всегда рады новым членам нашего сообщества Кухня Роботов.
    У нас есть бесплатные вводные курсы, где мы объясняем
    как работать с нашими образовательными наборами по робототехнике и электронике.
    ****************************************************
    Название набора: РобоРука Р1
    Суть программы: Прием координат от пользователя и установка робота-манипулятора в нужное положение.
    Программа создана и протестирована разработчиком:
    Имя: Максим Массальский
    Ник: maxxlife
    E-mail: maxxliferobot@gmail.com
    В программе используются сторонние коды от разработчика Oleg Mazurov
    https://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
*/
#include <Wire.h>
#include <Multiservo.h>

/* Размеры робота */
double base_height = 80; //Высота базовой платформы от оси вращения Плечевого сустава до земли
double humerus = 146;    //Расстояние от оси вращения плечевого сустава до оси вращения локтевого сустава
double ulna = 125;     //Расстояние от оси вращения локтевого сустава до оси вращения кистевого сустава

//Расстояние от оси вращения кистевого сустава до кончика Клешни-захвата РобоКлешня X1S
//Закомментировать одно из двух gripperpper для нужного случая
//double gripperpper=185;
//Расстояние от оси вращения кистевого сустава до кончика Клешни-захвата РобоКлешня Z1S
double gripper = 145;

/* Распиновка сервоприводов к сервойдрайверу */
/* Сервопривод поворота основания */
int base_servo = 0;
/* Сервопривод плечевого сустава*/
int shld_servo = 1;
/* Сервопривод локтевого сустава*/
int elbow_servo = 2;
/* Сервопривод кистевого сустава */
int wrist_servo = 3;
/* Сервопривод захвата-клешни */
int gripper_servo = 4;

/* Предварительные вычисления */
double hum_sq = humerus * humerus;
double uln_sq = ulna * ulna;

Multiservo servo_base;
Multiservo servo_shld;
Multiservo servo_elbow;
Multiservo servo_wrist;
Multiservo servo_gripper;

//Минимальные значения для сервоприводов по часовой стрелке
int base_servopulse_min_cw = 600;
int shld_servopulse_min_cw = 2200;
int elbow_servopulse_min_cw = 750;
int wrist_servopulse_min_cw = 600;

//Максимальные значения для сервоприводов против часовой стрелки
int shld_servopulse_max_ccw = 600;
int elbow_servopulse_max_ccw = 2250;
int wrist_servopulse_max_ccw = 2400;
int base_servopulse_max_ccw = 2400;

//Значения для открытого и закрытого захвата-клешни X1S
//int gripper_servopulse_open = 700;
//int gripper_servopulse_closed = 1650;
//Значения для открытого и закрытого захвата-клешни Z1S
int gripper_servopulse_open = 500;
int gripper_servopulse_closed = 1500;

//Переменные для хранения длительности импульсов в микросекундах
int base_servopulse_prev = 0;
int shld_servopulse_prev = 0;
int elbow_servopulse_prev = 0;
int wrist_servopulse_prev = 0;

//Длительности импульсов для Начального положения
int base_servopulse_down = 1500;//1600
int shld_servopulse_down = 2200;
int elbow_servopulse_down = 2200;
int wrist_servopulse_down = 1500;//1550

//Длительности импульсов для Стартового положения
int base_servopulse_90 = 1500;//1600
int shld_servopulse_90 = 1500;
int elbow_servopulse_90 = 1500;
int wrist_servopulse_90 = 1500;//1550

//Переменные для хранения длительности импульсов в микросекундах
int base_servopulse = 0;
int shld_servopulse = 0;
int elbow_servopulse = 0;
int wrist_servopulse = 0;
int gripper_servopulse = 0;

// Для ввода данных переменная, которая будет в себе хранить лишние символы
char junk = ' ';

double x_enter = 0;
double y_enter = 0;
double z_enter = 0;
double gripper_enter = 0;
int right_enter = 0;
// Функция setup() выполняется каждый раз, когда будет перезапущена подача питания
// или будет произведена перезагрузка платы
void setup()
{
  Wire.begin();
  Serial.begin( 9600);
  //Прикрепление сервоприводов к определенным пинам на серводрайвере
  servo_base.attach(0);
  servo_shld.attach(1);
  servo_elbow.attach(2);
  servo_wrist.attach(3);
  servo_gripper.attach(4);
  //Функция servo_start устанавливает сервопривод в положение Режима ожидания
  delay(2000);
  servo_start();
  Serial.println("Let's play with the arm P1!");
    delay(30000);
}

bool state = true;
void loop()
{
  //Функция по нахождению координат и изменению положения робота-манипулятора
  //Убедись, что у тебя в Мониторе последовательного порта скорость 9600 и стоит Не найден конец строки!!!
  find_coordinates();
}

//Функция find_coordinates позволяет пользователю вводит координаты точки
//и смотреть какое положение примет робот-манипулятора.
void find_coordinates()
{
label1:
  Serial.println("Enter value for X. Press ENTER");
  while (Serial.available() == 0) ;
  {
    x_enter = Serial.parseFloat();
    Serial.print("x = "); Serial.println(x_enter, DEC);

    while (Serial.available() > 0)
    {
      junk = Serial.read() ;
    }
  }

  Serial.println("Enter value for Y. Press ENTER");
  while (Serial.available() == 0) ;
  {
    y_enter = Serial.parseFloat();
    Serial.print("y = "); Serial.println(y_enter, DEC);
    while (Serial.available() > 0)
    {
      junk = Serial.read() ;
    }
  }

  Serial.println("Enter value for Z. Press ENTER");
  while (Serial.available() == 0) ;
  {
    z_enter = Serial.parseFloat();
    Serial.print("z = "); Serial.println(z_enter, DEC);
    while (Serial.available() > 0)
    {
      junk = Serial.read() ;
    }
    gripper_enter = 0;
  }

  //Раскомментировать, если необходимо указывать  угол наклона гриппера.
  //Может понадобится, когда предмет надо брать сверху и т.д.
  /*
    Serial.println("Enter value for gripper Angle. Press ENTER");
    while (Serial.available() > 0) ;
    {
      gripper_enter = Serial.parseFloat();
      Serial.print("Gripper Angle = ");
      Serial.println(gripper_enter, DEC);
      while (Serial.available() > 0)
      {
        junk = Serial.read() ;
      }
    }
  */
  Serial.println("If coordinates are correct Enter digit 1. If incorrect Enter 0 to re-enter data. Press ENTER.");
  while (Serial.available() == 0) ;
  {
    right_enter = Serial.parseInt();
    if (right_enter == 1)
    {
      Serial.println("Start set arm");
      Serial.println(x_enter);
      Serial.println(y_enter);
      Serial.println(z_enter);
      Serial.println(gripper_enter);
      set_arm(x_enter, y_enter, z_enter, gripper_enter);// Опускаем захват, ставя точку.
    }
    else
    {
      goto label1;
    }
    while (Serial.available() > 0)
    {
      junk = Serial.read() ;
    }
  }
}

/* Функция по позиционированию робота-манипулятора с использованием Инверсной кинематики*/
//Координата x может быть отрицательной и положительной.
//Координаты y и z могут быть только положительные.
//Подробнее про вычисления читай здесь https://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
//И здесь http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf

void set_arm( double x, double y, double z, double gripper_angle_d )
{
  //Вычисления углов наклона звеньев робота-манипулятора, зная координаты точки.
  //Все углы находятся в радианах, а потом конвертируются.
  double gripper_angle_r = radians( gripper_angle_d );   //Перевод в радианы

  /* Угол поворота основания*/
  double base_angle_r = atan2( x, y );
  //Расстояние, зная x и y.
  double rdist = sqrt(( x * x ) + ( y * y ));
  /* rdist это координата y для манипулятора*/
  y = rdist;
  /* Офсеты для гриппера, зная угод его наклона относительно земли. */
  double gripper_off_z = ( sin( gripper_angle_r )) * gripper;
  double gripper_off_y = ( cos( gripper_angle_r )) * gripper;
  /* Позиция кисти*/
  double wrist_z = ( z - gripper_off_z ) - base_height;
  double wrist_y = y - gripper_off_y;
  /* Расстояние от оси поворота плечевого сустава до оси поворота кистевого сустава */
  double s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  double s_w_sqrt = sqrt( s_w );
  /* s_w угол относительно земли*/
  //double a1 = atan2( wrist_y, wrist_z );
  double a1 = atan2( wrist_z, wrist_y );
  /* s_w угол относительно humerus */
  double a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * humerus * s_w_sqrt ));
  /* Угол наклона плечевого сустава*/
  double shld_angle_r = a1 + a2;
  double shld_angle_d = degrees( shld_angle_r );
  /* Угол наклона локтевого сустава */
  double elbow_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * humerus * ulna ));
  //Угол наклона локтевого сустава в градусах
  double elbow_angle_d = degrees( elbow_angle_r );
  double elbow_angle_dn = -( 180.0 - elbow_angle_d );
  /* Угол наклона кисти в градусах*/
  double wrist_angle_d = ( gripper_angle_d - elbow_angle_dn ) - shld_angle_d;
  //Угол поворота основания в градусах
  double base_angle_d = degrees(base_angle_r);
  shld_angle_d = shld_angle_d - 90;
  elbow_angle_d = elbow_angle_d - 90;
  Serial.println(" ");
  Serial.println(shld_angle_d);
  Serial.println(elbow_angle_d);
  Serial.println(wrist_angle_d);
  Serial.println(base_angle_d);
  Serial.println(" ");
  /* Вычисления длительности импульсов*/
  //Числа 9 и 10 являются коэффициентами для сервоприводов
  //Они позволяются узнать длительности импульса, зная угол поворота сервоприводов
  //Т.е. они показывают зависимость угла поворота от длительности импульса.
  base_servopulse = base_servopulse_90 - ( base_angle_d * 10 );
  shld_servopulse = shld_servopulse_90 + ( shld_angle_d  * 9 );
  elbow_servopulse = elbow_servopulse_90 -  ( elbow_angle_d * 9 );
  wrist_servopulse = wrist_servopulse_90 - ( wrist_angle_d * 10);

  //Магический while по плавному изменению угла поворота сервопривода.
  //Т.к. российский драйвер производства Амперка это делать не умеет.
  while ((base_servopulse_prev > base_servopulse) || (base_servopulse_prev < base_servopulse)
         || (shld_servopulse_prev > shld_servopulse) || (shld_servopulse_prev < shld_servopulse)
         || (elbow_servopulse_prev > elbow_servopulse) || (elbow_servopulse_prev < elbow_servopulse)
         || (wrist_servopulse_prev > wrist_servopulse) || (wrist_servopulse_prev < wrist_servopulse)
        )
  {
    if ((base_servopulse_prev <= base_servopulse + 10) && (base_servopulse_prev >= base_servopulse - 10))
      base_servopulse_prev = base_servopulse;
    if (base_servopulse_prev > base_servopulse)
      base_servopulse_prev = base_servopulse_prev - 5;
    if (base_servopulse_prev < base_servopulse)
      base_servopulse_prev = base_servopulse_prev + 5;

    if ((shld_servopulse_prev <= shld_servopulse + 10) && (shld_servopulse_prev >= shld_servopulse - 10))
      shld_servopulse_prev = shld_servopulse;
    if (shld_servopulse_prev > shld_servopulse)
      shld_servopulse_prev = shld_servopulse_prev - 5;
    if (shld_servopulse_prev < shld_servopulse)
      shld_servopulse_prev = shld_servopulse_prev + 5;

    if ((elbow_servopulse_prev <= elbow_servopulse + 10) && (elbow_servopulse_prev >= elbow_servopulse - 10))
      elbow_servopulse_prev = elbow_servopulse;
    if (elbow_servopulse_prev > elbow_servopulse)
      elbow_servopulse_prev = elbow_servopulse_prev - 5;
    if (elbow_servopulse_prev < elbow_servopulse)
      elbow_servopulse_prev = elbow_servopulse_prev + 5;

    if ((wrist_servopulse_prev <= wrist_servopulse + 10) && (wrist_servopulse_prev >= wrist_servopulse - 10))
      wrist_servopulse_prev = wrist_servopulse;
    if (wrist_servopulse_prev > wrist_servopulse)
      wrist_servopulse_prev = wrist_servopulse_prev - 5;
    if (wrist_servopulse_prev < wrist_servopulse)
      wrist_servopulse_prev = wrist_servopulse_prev + 5;

    servo_base.writeMicroseconds(base_servopulse_prev);
    servo_shld.writeMicroseconds(shld_servopulse_prev);
    servo_elbow.writeMicroseconds(elbow_servopulse_prev);
    servo_wrist.writeMicroseconds(wrist_servopulse_prev);
    //Необходимая задержка для плавности хода.
    delay(15);
  }
}

/* Функция, которая ставит все сервоприводы в положение Режима ожидания команд*/
void servo_park_90()
{
  servo_base.writeMicroseconds(base_servopulse_90);
  servo_shld.writeMicroseconds(shld_servopulse_90);
  servo_elbow.writeMicroseconds(elbow_servopulse_90);
  servo_wrist.writeMicroseconds(wrist_servopulse_90);
  servo_gripper.writeMicroseconds(gripper_servopulse_closed);
  base_servopulse_prev = base_servopulse_90;
  shld_servopulse_prev = shld_servopulse_90;
  elbow_servopulse_prev = elbow_servopulse_90;
  wrist_servopulse_prev = wrist_servopulse_90;

  return;
}
void servo_park_down()//Собраное положение манипулятора
{
  servo_base.writeMicroseconds(base_servopulse_down);//2400 left 600 right
  servo_shld.writeMicroseconds(shld_servopulse_down);//600 down, up 2200
  servo_elbow.writeMicroseconds(elbow_servopulse_down);//2250 down, up750
  servo_wrist.writeMicroseconds(wrist_servopulse_down);//2400 down,up 600
  servo_gripper.writeMicroseconds(gripper_servopulse_closed);//open 700, closed 1550
  base_servopulse_prev = base_servopulse_down;
  shld_servopulse_prev = shld_servopulse_down;
  elbow_servopulse_prev = elbow_servopulse_down;
  wrist_servopulse_prev = wrist_servopulse_down;
  return;
}

void servo_start()
{
  servo_park_down();
  delay(1000);
  while ((shld_servopulse_prev > shld_servopulse_90) || (elbow_servopulse_prev > elbow_servopulse_90) || (wrist_servopulse_prev > wrist_servopulse_90))
  {
    //if нужен, чтобы не со всех переменных отнимать значения, вдруг переменная уже имеет нужное значение.
    if (shld_servopulse_prev > shld_servopulse_90)
      shld_servopulse_prev = shld_servopulse_prev - 10;

    if (elbow_servopulse_prev > elbow_servopulse_90)
      elbow_servopulse_prev = elbow_servopulse_prev - 10;

    if (wrist_servopulse_prev > wrist_servopulse_90)
      wrist_servopulse_prev = wrist_servopulse_prev - 10;
    servo_shld.writeMicroseconds(shld_servopulse_prev);
    servo_elbow.writeMicroseconds(elbow_servopulse_prev);
    servo_wrist.writeMicroseconds(wrist_servopulse_prev);
    delay(40);
  }
}

