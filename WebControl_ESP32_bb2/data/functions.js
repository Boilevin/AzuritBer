// Abfrageintervall der Werte in ms
var abfrageIntervall = 1000;
// nicht mehrere Anfragen parallel senden
var ajaxAktiv=false;
// beim Neuladen parameter komplett anfordern
var neuGeladen = true;

var reqCmd = "";

var aktivity = 0; //Requestzähler für Aktivitätsanzeige

var dragID="";  //ID des Sliders der gerade verändert wird (zum ausblenden der aktualisierung)

//Verbindungsanzeige
function dispCon(state){
  var ele = document.getElementById("con");
  ele.innerText = state;
}

//Progress-Anzeige aktualisierung
function writeProgress(){
  var ele = document.getElementById("aktivity");
  switch (aktivity) {
    case 0:
      ele.innerText=" < | | | | | | |";
      break;
    case 1:
      ele.innerText=" . < | | | | | |";
      break;
    case 2:
      ele.innerText=" . . < | | | | |";
      break;
    case 3:
      ele.innerText=" , . . < | | | |";
      break;
	case 4:
      ele.innerText=" , , . . < | | |";
      break;
    case 5:
      ele.innerText=" | , , . . < | |";
      break;
    case 6:
      ele.innerText=" | | , , . . < |";
      break;
	case 7:
      ele.innerText=" | | | , , . . <";
      break;
	case 8:
      ele.innerText=" | | | | , , . .";
      break;
	case 9:
      ele.innerText=" | | | | | , , .";
      break; 
	case 10:
      ele.innerText=" | | | | | | , ,";
      break;      
	case 11:
      ele.innerText=" | | | | | | | ,";
      break;
    case 12:
      ele.innerText=" | | | | | | | |";
      break;    
    default:
  }

  if (aktivity>=12)aktivity = 0;
  else aktivity++;
}

// Die Keys heißen wie die Felder
function writeAntwort(str) {
var ele;
var lbCount = 1;
writeProgress();

  //  an & trennen
  var arr = str.split('&');
  for(var i=0; i<arr.length; i++) {
    // an = trennen
    var keyVal = arr[i].split('=');
    var keyPart = keyVal[0].split("_");
    if (keyPart.length == 2){
      //ist ein zusammengesetztes Element (slider)
      switch (keyPart[1]) {
        case "val":
          setSliderVal(keyPart[0],keyVal[1])
          break;
        case "min":
          setSliderMin(keyPart[0],keyVal[1])
          break;
        case "max":
          setSliderMax(keyPart[0],keyVal[1])
          break;
        case "res":
          setSliderRes(keyPart[0],keyVal[1])
          break;
        default:
      }
    } else {
      //element ist ein textfeld oder ein button
      if (keyVal[0] == "con")dispCon(keyVal[1]);
      else {
        ele = document.getElementById(keyVal[0]);
        if (ele == null){
          ele = document.getElementById(keyVal[0] + "_" + lbCount);
          lbCount++;
        }
        if (ele != null){
          ele.innerText = keyVal[1];
        }
      }
    }
  }

}

function setSliderVal(id,val) {
  if (dragID != id){
    var objSlider, objMin, objMax, objVal, objAct;
    objSlider = document.getElementById(id);
    if (Number(val) < Number(objSlider.min))objSlider.value = objSlider.min;
    else if (Number(val) > Number(objSlider.max))objSlider.value = objSlider.max;
    else objSlider.value = val;

    objVal = document.getElementById(id + "_val");
    objVal.innerText = val;
    objAct = document.getElementById(id + "_act");
    objAct.innerText = val;
  }
}

function setSliderMin(id,min) {
  var objSlider, objMin, objMax, objVal, objAct;
  objSlider = document.getElementById(id);
  objSlider.min = min;

  objMin = document.getElementById(id + "_min");
  objMin.innerText = min;
}

function setSliderMax(id,max) {
  var objSlider, objMin, objMax, objVal, objAct;
  objSlider = document.getElementById(id);
  objSlider.max = max;
  objMax = document.getElementById(id + "_max");
  objMax.innerText = max;
}

function setSliderRes(id,res) {
  var objSlider, objMin, objMax, objVal, objAct;
  objSlider = document.getElementById(id);
  objSlider.step = res;
}

function setSlider(id,min,max,val) {
  var objSlider, objMin, objMax, objVal, objAct;
  objSlider = document.getElementById(id);
  objSlider.value = val;
  objSlider.min = min;
  objSlider.max = max;

  objMin = document.getElementById(id + "_min");
  objMin.innerText = min;
  objMax = document.getElementById(id + "_max");
  objMax.innerText = max;
  objVal = document.getElementById(id + "_val");
  objVal.innerText = val;
  objAct = document.getElementById(id + "_act");
  objAct.innerText = val;
}

// holt die Werte vom Server
function getWerte(requ) {
  reqCmd = requ;

  // Warten, bis frei
  while(ajaxAktiv);
  ajaxAktive = true;
  // AJAX
  xhr = new XMLHttpRequest();
  //xhr.open('GET', '/ardu/ajax.php');
  if (neuGeladen){
    xhr.open('POST', '/werte?cmd=' + requ + '&refresh=false');
  } else {
    xhr.open('POST', '/werte?cmd=' + requ + '&refresh=true');
  }

  xhr.onload = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        // console.log('Antwort OK ' + xhr.responseText);
        writeAntwort(xhr.responseText);
      }
      else if (xhr.status !== 200) {
        console.log('Request failed.  Returned status of ' + xhr.status);
      }
      ajaxAktiv=false;
    }
  };
  xhr.send(null);

  if (neuGeladen) {
    neuGeladen = false;
    setInterval(function(){ getWerte(reqCmd) },abfrageIntervall); // aller x Sekunden Werte holen
  }

}

// Sendeparameter zusammenfassen und AJAX-Aufruf durchführen
function buildParams(paramStr) {
  // warten bis AJAX-Request fertig
  while(ajaxAktiv);
  ajaxAktive = true;

  //console.log('ParamStr: '+ paramStr);
  // AJAX
  xhr = new XMLHttpRequest();

  xhr.open('POST', '/set?' + paramStr);

  xhr.onload = function() {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        // console.log('Antwort OK ' + xhr.responseText);
        writeAntwort(xhr.responseText);
      }
      else if (xhr.status !== 200) {
        console.log('Request failed.  Returned status of ' + xhr.status);
      }
      ajaxAktiv=false;
      sliderSend = false;
    }
  }
  xhr.send(null);

  if (neuGeladen) {
    neuGeladen = false;
    setInterval(function(){ getWerte(reqCmd) },abfrageIntervall); // aller x Sekunden Werte holen
  }

}

//Button wurde angeklicked
function isClicked(id) {
    buildParams(id + "=true");
}

//Slider wurde verändert und losgelassen
function isChange(id) {
  dragID = "";
  var val,objSlider, objVal,step;
  var changed = false;
  objSlider = document.getElementById(id);
  objVal = document.getElementById( id + "_val");
  val = objSlider.value;
  step = objSlider.step;
  if (step == "")step = 1;

  // Hat sich was geändert
  if (val != objVal.innerText) {
    buildParams(id + "=" + Math.round(val/step));
    objVal.innerText = val;
  }
}

//Slider wird verändert
function isChanging(elem){
  dragID = elem;

  var ID = elem;
  var val,ele, ele1;
  var changed = false;
  ele = document.getElementById(ID);
  ele1 = document.getElementById( ID + "_act");
  val = ele.value;
  ele1.innerText = val;
}