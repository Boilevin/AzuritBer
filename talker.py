# pylint: disable=wildcard-import,unused-wildcard-import
from ..nmea import TalkerSentence
from ..nmea_utils import *
from ..seatalk_utils import *

from collections import namedtuple
from decimal import Decimal


#pylint: disable=missing-docstring
#pylint: disable=no-init
#pylint: disable=too-few-public-methods

class TOW(TalkerSentence):
    """ message from tower NANO to PI  for laser and rain sensor etc....
    """
    fields = (
        ("Sensor1", "sensor1"),
        ("Sensor2", "sensor2"),
        ("Sensor3", "sensor3"),
        ("Sensor4", "sensor4"),
        ("Sensor5", "sensor5"),
        ("Sensor6", "sensor6"),
        ("Rain", "rain"),
        
    )

class BYL(TalkerSentence):
    """ non stop message from DUE to PI  for perimater plot and visu etc....
    """
    fields = (
        ("Millis", "millis"),
        ("rollDir", "rollDir"),
        ("laneInUse", "laneInUse"),
        ("YawActual", "YawActual"),
        ("YawCible", "YawCible"),
    )

class PER(TalkerSentence):
    """ non stop message from DUE to PI  for perimater plot and visu etc....
    """
    fields = (
        ("Millis", "millis"),
        ("perimeterMag", "perimeterMag"),
        ("perimeterMagRight", "perimeterMagRight"),
        ("AreaInMowing","areaInMowing"),
		
    )
    
class BAT(TalkerSentence):
    """ non stop message from DUE to PI  for battery plot and visu etc....
    """
    fields = (
        ("Millis", "millis"),
        ("batVoltage", "batVoltage"),
        ("chgVoltage", "chgVoltage"),
	("chgSense", "chgSense"),
	
    )
    
class IMU(TalkerSentence):
    """ non stop message from DUE to PI  for imu plot and visu etc....
    """
    fields = (
        ("Millis", "millis"),
        ("gyroYaw", "gyroYaw"),
        ("compassYaw", "compassYaw"),
	
	
    )

class MOT(TalkerSentence):
    """ non stop message from DUE to PI  for wheel motor plot and visu etc....
    """
    fields = (
        ("Millis", "millis"),
        ("motorLeftSenseCurrent", "motorLeftSenseCurrent"),
        ("motorRightSenseCurrent", "motorRightSenseCurrent"),
	("motorLeftPWMCurr", "motorLeftPWMCurr"),
	("motorRightPWMCurr", "motorRightPWMCurr"),
        ("BatVoltage", "batVoltage"),
        
		
    )
class MOW(TalkerSentence):
    """ non stop message from DUE to PI  for mow motor plot and visu etc....
    """
    fields = (
        ("Millis", "millis"),
        ("motorMowSense", "motorMowSense"),
        ("motorMowPWMCurr", "motorMowPWMCurr"),
        ("BatVoltage", "batVoltage"),
        
		
    )

class STA(TalkerSentence):
    """ non stop message from DUE to PI  for state, localisation etc....
    """
    fields = (
        ("Millis", "millis"),
        ("State", "state"),
        ("OdometryX", "odox"),
	("OdometryY", "odoy"),
	("PrevYaw", "prevYaw"),
        ("BatVoltage", "batVoltage"),
        ("ActualYaw", "yaw"),
        ("ActualPitch", "pitch"),
        ("ActualRoll", "roll"),
        ("DHT22Temp", "Dht22Temp"),
        ("loopsPerSecond", "loopsPerSecond"),
        
    )    
class STU(TalkerSentence):
    """ message from DUE to PI  for status send only when change etc....
    """
    fields = (
        ("Status", "status"),
        ("Val1", "val1"),
	("Val2", "val2"),
	("Val3", "val3"),
	("Val4", "val4"),
	("Val5", "val5"),
	("Val6", "val6"),
	("Val7", "val7"),
	("Val8", "val8"),
	("Val9", "val9"),
	("Val10", "val10"),
		
    )

class RFI(TalkerSentence):
    """ message from DUE to PI  for rfid and status send only when tag is detected....
    """
    fields = (
        ("Status", "status"),
        ("rfidTagFind", "rfidTagFind"),
	("Val2", "val2"),
	("Val3", "val3"),
	("Val4", "val4"),
	("Val5", "val5"),
	("Val6", "val6"),
	("Val7", "val7"),
	("Val8", "val8"),
	("Val9", "val9"),
	("Val10", "val10"),
		
    )
class DEB(TalkerSentence):
    """ message from DUE TO PI for  debug etc....
    """
    fields = (
        ("Hubtype", "Hubtype"),
        ("Debug", "debug"),
    )


class PFO(TalkerSentence):
    """ message between pi and due with same def as pfod
    """
    fields = (
        ("Command", "command"),
        ("Val1", "val1"),
	("Val2", "val2"),
	("Val3", "val3"),
	("Val4", "val4"),
	("Val5", "val5"),
	("Val6", "val6"),
	("Val7", "val7"),
	("Val8", "val8"),
	("Val9", "val9"),
	("Val10", "val10"),
		
    )

class SET(TalkerSentence):
    """ message between pi and due to request all the setting of setting_page, by page of 10 values
    """
    fields = (
        ("Setting_page", "setting_page"),
        ("ReadWrite", "readwrite"),
	("Nb_page", "nb_page"),
        ("Val1", "val1"),
	("Val2", "val2"),
	("Val3", "val3"),
	("Val4", "val4"),
	("Val5", "val5"),
	("Val6", "val6"),
	("Val7", "val7"),
	("Val8", "val8"),
	("Val9", "val9"),
	("Val10", "val10"),
		
    )

class REQ(TalkerSentence):
    """ pi request information on DUE 
    """
    fields = (
        ("Message_type", "message_type"),
       	("Frequency", "frequency"),
	("Trigger", "trigger"),
	("Max_repetiton", "max_repetition"),
	("Val4", "val4"),
	("Val5", "val5"),
	("Val6", "val6"),
	("Val7", "val7"),
	
		
    )
class INF(TalkerSentence): #info return by due
    fields = (
        ("Millis", "millis"),
        ("Version", "version"),
       	("DeveloperActive", "developerActive"),
	("StatsOverride", "statsOverride"),
	("StatsMowTimeMinutesTrip", "statsMowTimeMinutesTrip"),
        ("StatsMowTimeHoursTotal", "statsMowTimeHoursTotal"),
	("StatsBatteryChargingCounterTotal", "statsBatteryChargingCounterTotal"),
        ("StatsBatteryChargingCapacityTrip", "statsBatteryChargingCapacityTrip"),
	("StatsBatteryChargingCapacityTotal", "statsBatteryChargingCapacityTotal"),
	("StatsBatteryChargingCapacityAverage", "statsBatteryChargingCapacityAverage"),
	
		
		
    )  
class RET(TalkerSentence):
    """ immediate response from DUE to PI to send all the setting of request in setting mode by page of 10 values
    $RMRET,Timer,0,1,13,37,15,23,0,1,3,0,34,*30
"""
    fields = (
        ("Setting_page", "setting_page"),
        ("PageNr", "pageNr"),
	("Val1", "val1"),
	("Val2", "val2"),
	("Val3", "val3"),
	("Val4", "val4"),
	("Val5", "val5"),
	("Val6", "val6"),
	("Val7", "val7"),
	("Val8", "val8"),
	("Val9", "val9"),
	("Val10", "val10"),
             
        
		
    )
    
class CMD(TalkerSentence):
    """ Use to SET or rest an actuator
    """
    fields = (
        ("ActuatorName", "actuatorname"),
       	("Val1", "val1"),
	("Val2", "val2"),
	("Val3", "val3"),
	("Val4", "val4"),
	
		
    )
    
class VAR(TalkerSentence):
    """ read or write follow by 4 time variable name follow by value to directly change something 0 to variable not use
    """
    fields = (
        ("ReadWrite", "readwrite"),
        ("Var1Name", "var1name"),
        ("Var1Value", "var1value"),
        ("Var2Name", "var2name"),
        ("Var2Value", "var2value"),
        ("Var3Name", "var3name"),
        ("Var3Value", "var3value"),
        ("Var4Name", "var4name"),
        ("Var4Value", "var4value"),
        ("EndValue", "endvalue"),
        
       
    )
    
class RMC(TalkerSentence):
    """ Recommended Minimum Specific GPS/TRANSIT Data
    """
    fields = (
        ("Timestamp", "timestamp"),
        ('Status', 'status'), # contains the 'A' or 'V' flag
        ("Latitude", "lat"),
        ("Latitude Direction", "lat_dir"),
        ("Longitude", "lon"),
        ("Longitude Direction", "lon_dir"),
        ("Speed Over Ground", "spd_over_grnd", float),
        ("True Course", "true_course", float),
        ("Datestamp", "datestamp"),
        ("Magnetic Variation", "mag_variation"),
        ("Magnetic Variation Direction", "mag_var_dir"),
    )

