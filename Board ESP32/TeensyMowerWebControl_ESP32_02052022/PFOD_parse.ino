//Hauptmenü {.}
//{.Ardumower (Ardumower)|r~Commands|n~Manual|s~Settings|in~Info|c~Test compass|m1~Log sensors|yp~Plot|y4~Error counters|y9~ADC calibration}

//Settings {s}
//{.Settings|sz~Save settings|s1~Motor|s2~Mow|s3~BumperDuino|s4~Sonar|s5~Perimeter|s6~Lawn sensor|s7~IMU|s8~R/C|s9~Battery|s10~Station|s11~Odometry|s13~Rain|s15~Drop sensor|s14~GPS|i~Timer|s12~Date/time|sx~Factory settings}

//Motor-Settings {s1}
//{.Motor`1000|a00~Overload Counter l, r 0, 0|a01~Power in Watt l, r 0.00, 0.00|a05~motor current in mA l, r 0.00, 0.00|a02~Power max `750`1000`0~ ~0.1|a03~calibrate left motor  `0`1000`0~ ~1|a04~calibrate right motor `0`1000`0~ ~1|a05~Speed l, r pwm0.00, 0.00|a15~Speed max in pwm `255`255`0~ ~1|a11~Accel `1000`2000`500~ ~1|a18~Power ignore time `2000`8000`0~ ~1|a07~Roll time max `1500`8000`0~ ~1|a19~Roll time min `750`1000`0~ ~1|a08~Reverse time `1200`8000`0~ ~1|a09~Forw time max `12000`8000`0~ ~10|a12~Bidir speed ratio 1 `30`100`0~ ~0.01|a13~Bidir speed ratio 2 `92`100`0~ ~0.01|a10~Testing isOFF|a14~for config file:motorSenseScale l, r6.15, 6.15|a16~Swap left direction NO|a17~Swap right direction NO}

//Batterie-Settings {s9}
//{.Battery`1000|j00~Battery 29.37 V|j01~Monitor YES|j02~Go home if below Volt `237`293`211~ ~0.1|j12~Switch off if idle minutes `0`300`1~ ~1|j03~Switch off if below Volt `217`293`211~ ~0.1|j04~Charge 0.06V 0.00A|j08~Charge factor `16`9`59~ ~0.001|j10~charging starts if Voltage is below `320`293`0~ ~0.1|j11~Battery is fully charged if current is below `10`16`0~ ~0.1}

//Sonar Settings
//{.Sonar`1000|d00~Use YES|d04~Use left YES|d05~Use center YES|d06~Use right YES|d01~Counter 0|d02~Value l, c, r 95, 105, 0|d03~Trigger below (0=off) `1797`3000`0~ ~1|d07~Slow below `1115`3000`0~ ~1}

void parse_PFOD(String s) {
  String element = "";
  String params = "";
  String temp;
  Serial.println(s);

  if (!s.startsWith("{")) {
    s = s.substring(s.indexOf("{"), s.indexOf("}") + 1);
  }

  if (!s.startsWith("{") && !s.endsWith("}")) return;

  debugln("Parse String: " + s);
  s.replace("{", "");
  s.replace("}", "");
  //s.replace("\n", "");
  //s.replace("\r", "");

  nElemente = count(s, '|') + 1;

  for (int i = 0; i < nElemente; i++) {
    element = split(s, '|', i);

    //Element auswerten
    if (count(element, '~') == 0) {
      //es handelt sich um die Menü-Überschrift
      elemente[i].cmd = ".";
      if (count(element, '`') > 0) {
        temp = split(element, '`', 0);
        elemente[i].value = split(element, '`', 1); //Refresh-Time
      }
      else temp = element;
      temp.replace(".", "");
      elemente[i].title = temp;
      elemente[i].type = MENU;

    } else if (count(element, '~') == 1) {
      //es handelt sich um ein text-Element (bzw. link)
      elemente[i].cmd = split(element, '~', 0);
      elemente[i].title = split(element, '~', 1);
      elemente[i].value = split(element, '~', 1);
      elemente[i].type = LINK;

    } else if (count(element, '~') == 3) {
      //es handelt sich um ein Slider-Element
      String params = split(element, '~', 1);

      elemente[i].cmd = split(element, '~', 0);
      elemente[i].title = split(params, '`', 0);
      elemente[i].sl_res = split(element, '~', 3);
      elemente[i].value = getSkaleVal(split(params, '`', 1), elemente[i].sl_res);
      elemente[i].sl_max = getSkaleVal(split(params, '`', 2), elemente[i].sl_res);
      elemente[i].sl_min = getSkaleVal(split(params, '`', 3), elemente[i].sl_res);

      elemente[i].type = SLIDER;
    }

    elemente[i].title.replace("=", ":");
    elemente[i].value.replace("=", ":");

    debug("CMD " + elemente[i].cmd);
    debug(" ,title " + elemente[i].title);
    debug(" ,value " + elemente[i].value);
    debug(" ,sl_min " + elemente[i].sl_min);
    debug(" ,sl_max " + elemente[i].sl_max);
    debug(" ,sl_res " + String(elemente[i].sl_res));
    debugln(" ,typ " + String(elemente[i].type));
  }

}

String getSkaleVal(String value, String skale) {

  float fValue = value.toFloat();
  float fskale = skale.toFloat();
  float wert = 0.0;
  String erg = "";
  wert = fValue * fskale;

  if (fskale >= 1.0) {
    erg = String(long(wert));
  } else if (fskale == 0.001) {
    erg = String(wert, 3);
  } else if (fskale == 0.01) {
    erg = String(wert, 2);
  } else if (fskale == 0.1 ) {
    erg = String(wert, 1);
  } else {
    erg = String(wert, 1);
  }

  //debugln("fskale=" + String(fskale, 5) + " fValue=" + String(fValue) + " erg=" + erg + " wert=" + wert);

  return erg;
}

String split(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int count(String s, char parser) {
  int parserCnt = 0;
  for (int i = 0; i < s.length(); i++) {
    if (s[i] == parser) parserCnt++;
  }
  return parserCnt;
}
