#include "RpiRemote.h"
#define N_FLOATS 4

uint16_t var_checksum;

String RpiCmd;


void RpiRemote::init() {
  Serial.begin(115200);

}

void RpiRemote::RaspberryPISendData () {
  //Serial.print(motorPowerMax);
  String lineToSend;
  lineToSend = "RMSTA,";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "123";
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}

void RpiRemote::writePi(String stringLine) {
  String lineToSend;
  uint8_t retour;
  retour = create_checksum(stringLine);
  lineToSend = "$" + stringLine + "*";
  if (retour < 16) {  //add the leading zero
    lineToSend = lineToSend + "0" + String(retour, HEX);
  }
  else {
    lineToSend = lineToSend + String(retour, HEX);
  }
  Serial.println(lineToSend);
  //Serial.println(lineToSend);
}


RpiRemote::Tokeniser::Tokeniser(char* _str, char _token)
{
  str = _str;
  token = _token;
}


bool RpiRemote::Tokeniser::next(char* out, int len)
{
  uint8_t count = 0;
  if (str[0] == 0)
    return false;

  while (true)
  {
    if (str[count] == '\0')
    {
      out[count] = '\0';
      str = &str[count];
      return true;
    }

    if (str[count] == token)
    {
      out[count] = '\0';
      count++;
      str = &str[count];
      return true;
    }

    if (count < len)
      out[count] = str[count];
    count++;
  }
  return false;
}

bool RpiRemote::encode(char c)
{
  buf[pos] = c;
  pos++;

  if (c == '\n') //linefeed
  {
    //Serial.println("..........FIND THE END LINE................");
    //Serial.println(buf);
    bool ret = process_buf();
    memset(buf, '\0', 120);
    pos = 0;
    return ret;
  }

  if (pos >= 120) //avoid a buffer overrun
  {
    Serial.print (buf);
    Serial.println("----------------- warning >120 char received  from PI------------");
    memset(buf, '\0', 120);
    pos = 0;
  }
  return false;
}


bool RpiRemote::process_buf()
{
  if (!check_checksum()) //if checksum is bad
  {
    //Serial.println("Error Checksum");
    //Serial.println (buf);
    return false; //return
  }
  else {
    //Serial.print("Checksum OK ");
    //Serial.print (buf);
    //otherwise, what sort of message is it
    if (strncmp(buf, "$RMPFO", 6) == 0) read_pfo();
    if (strncmp(buf, "$RMSET", 6) == 0) readWrite_setting();
    if (strncmp(buf, "$RMVAR", 6) == 0) readWrite_var();
    if (strncmp(buf, "$RMCMD", 6) == 0) receive_command();
    if (strncmp(buf, "$RMREQ", 6) == 0) receive_request();



    //here add the reading case of all type of command




    return true;
  }
}


uint8_t RpiRemote::create_checksum(String lineOfString)
{
  char lineOfChar[lineOfString.length() + 1];
  lineOfString.toCharArray(lineOfChar, lineOfString.length() + 1); //need a char array to compute

  uint8_t XOR = 0;
  for (uint8_t posit = 0; posit < strlen(lineOfChar); posit++) {
    XOR = XOR ^ lineOfChar[posit];
  }

  return XOR;

}



bool RpiRemote::check_checksum()
{

  if (buf[strlen(buf) - 5] == '*')
  {
    uint16_t sum = parse_hex(buf[strlen(buf) - 4]) * 16;
    sum += parse_hex(buf[strlen(buf) - 3]);
    //Serial.println(sum);
    var_checksum = sum;
    for (uint8_t i = 1; i < (strlen(buf) - 5); i++)
      sum ^= buf[i];
    if (sum != 0)
      return false;

    return true;
  }
  return false;
}


uint8_t RpiRemote::parse_hex(char c)
{
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A') + 10;
  return 0;
}



