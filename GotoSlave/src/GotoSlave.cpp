#define _GOTO_SLAVE_DEBUG_ON_  // Descomentar para habilitar depuración por Serial.

// Macros para facilitar la salida de información por el Serial.
#ifdef _GOTO_SLAVE_DEBUG_ON_
#define _GOTO_SLAVE_DEBUG_(type, text) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.println(text);
#define _GOTO_SLAVE_DEBUG_VALUE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value);
#define _GOTO_SLAVE_DEBUG_VALUE_DOUBLE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value, 15);
#else
#define _GOTO_SLAVE_DEBUG_(type, text) void();
#define _GOTO_SLAVE_DEBUG_VALUE_(type, text, value) void();
#define _GOTO_SLAVE_DEBUG_VALUE_DOUBLE_(type, text, value) void();
#endif


#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Librería para controlar el INA 3221 (Sensor de corriente)
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Librerias\Libreria SDL_Arduino_INA3221\SDL_Arduino_INA3221-master\SDL_Arduino_INA3221.cpp"

// Clase de control del telescopio.
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteTelesCtrlESPNOW\ToteTelesCtrlESPNOW_V1_1.cpp"

/**************************************************************************************************
 * MUY IMPORTANTE, COMPROBAR LOS DOS DEFINES QUE ESTÁN EN ToteESPNOWDeviceMode.h DE LA CLASE BASE *
 * (ToteESPNOW) ANTES DE COMPILAR                                                                 *
 **************************************************************************************************/

#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteGoto\ToteARGotoTMC_V1_1.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteGoto\ToteDECGotoTMC_V1_1.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteCommandFIFO\ToteCommandFIFO_V1_0.cpp"


// Prototipo de funciones.
void processMsg(void);	// Procesa los mensajes recibidos desde el CONTROLLER.
void peekCommands(void);    			// Procesa la cola de comandos.
void senseINA3221(void);				// Sondeo el INA3221.
void printReport(void);					// Muestra informe.
void printCurrent(void);				// Muestra datos sondeo INA3221.


// Comandos que se pueden encolar en la pila FIFO.
enum ExecuteCommandEnum {
	NO_COMMAND,
	AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP,
	AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP,
	AR_MOTOR_STOP,
	DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP,
	DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP,
	DEC_MOTOR_STOP,
	AR_DEC_MOTORS_GOTO_ORIGIN,
	AR_DEC_MOTORS_NEXT_ROSTER_VM,
	AR_MOTOR_TRACKING,
	AR_AXIS_FULL_REVOLUTION,
	DEC_AXIS_FULL_REVOLUTION
};

// Pines del ESP32 para controlar el driver del motor de Ascensión Recta.
#define AR_STEP_PIN		 		2
#define AR_ENABLE_PIN			4
#define AR_DIR_PIN  			15  
#define AR_CFG1_PIN  			18
#define AR_CFG2_PIN 			19

// Pines del ESP32 para controlar el driver del motor de Declinación.
#define DEC_STEP_PIN	 		25
#define DEC_ENABLE_PIN			14
#define DEC_DIR_PIN  			26  
#define DEC_CFG1_PIN  			33
#define DEC_CFG2_PIN 			32

// Pin del ESP32 para encender el LASER.
#define LASER_PIN 				27

// Velocidad máxima por defecto.
#define DEFAULT_AR_VM			80 	// Velocidad máxima en radianes/segundo para el motor de AR.
#define DEFAULT_DEC_VM			60	// Velocidad máxima en radianes/segundo para el motor de DEC.

// Prototipo de funciones
void IRAM_ATTR myARMotor_ISR(void);
void IRAM_ATTR myDECMotor_ISR(void);

// Estas variables están definidas en 'ToteESPNOW'. Son globales para facilitar el acceso a estas por las funciones de callback.
extern DataStruct sendingData;     		// Almacena datos que serán enviados al peer.
extern DataStruct* receivedDataPtr;   	// Almacena datos recibidos desde el peer.


// La reducción mecánica en el eje de AR es 1/R1 * 1/R2 (ver 'calculoDeRelaciones.xlsx) de forma que:
// R1 es la reducción del piñón del eje del motor y la corona acoplada al bisinfin, que es 16/48 = 1/3.
// R2 es la reducción interna de la montura, que para la CG4 es 130.
// En consecuencia, la reducción mecánica es:
double ARReduction = 1.0/3.0 * 1.0/130.0;

// La reducción mecánica en el eje de DEC es 1/R1 * 1/R2 (ver 'calculoDeRelaciones.xlsx) de forma que:
// R1 es la reducción del piñón del eje del motor y la corona acoplada al bisinfin, que es 16/48 = 1/3.
// R2 es la reducción interna de la montura, que para la CG4 es 65.
// En consecuencia, la reducción mecánica es:
double DECReduction = 1.0/3.0 * 1.0/65.0;

// Almacenará semáforo para tener en cuenta el giro de la Tierra en los gotos.
bool isComputeAries = true;  

// Pines SCL para INA3221
#define INA3221_SDA			21	// Datos por SCL.
#define INA3221_SCL			22	// Reloj.

// Periodo de sondeo del INA3221
#define INA3221_SENSE_PERIOD	1000	// Se sondea cada segundo.

// Última vez que se sondeó el INA3221.
unsigned long lastINASenseMillis;

// Estructura para almacenar lecturas de canales del INA3221
struct INA3221_DATA {
	float busVoltage;
  	float current_mA;
};

// Esta variable almacenará las lecturas de los tres canales del INA3211.
INA3221_DATA ina3221Data[3]; 



// Instancio objeto para sensar corriente de los motores y tensión de la batería.
SDL_Arduino_INA3221 myINA3221;

// stepPeriod = 1s, pulsePeriod = 4 us, timer = 0, Prescaler = 80.
ToteARGotoTMC myARMotor = ToteARGotoTMC(
									ARReduction, 															// theMechanicalReduction.
							  		ToteESP32MotorTimerTMC::TMCModeEnum::TMC_LEGACY, 						// theTMCMode.
									ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_2,	// theStepTypeLegacy.
							  		0.0, 																	// theAxisMinValue.		
									24.0, 																	// theAxisMaxValue.
									AR_ENABLE_PIN, 															// theEnablePin.
									AR_CFG1_PIN,															// thecfg1Pin.
									AR_CFG2_PIN, 															// thecfg2Pin.
									AR_STEP_PIN, 															// theStepPin.			
									AR_DIR_PIN, 															// theDirPin.
							 		1000000,																// theStepPeriod.
									20,																		// thePulsePeriod.
									0,																		// theHardwareTimer.
									&myARMotor_ISR,															// theISR. 
									80, 																	// thePrescaler.
									10, 																	// theA.
									10, 																	// theD.
									0, 																		// theVs.
									DEFAULT_AR_VM,															// theVm.
									200																		// theK.
									);


// stepPeriod = 1s, pulsePeriod = 4 us, timer = 0, Prescaler = 80.
ToteDECGotoTMC myDECMotor = ToteDECGotoTMC(
									DECReduction, 															// theMechanicalReduction.
							  		ToteESP32MotorTimerTMC::TMCModeEnum::TMC_LEGACY, 						// theTMCMode.
									ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_2,	// theStepTypeLegacy.
							  		0.0, 																	// theAxisMinValue.		
									360.0, 																	// theAxisMaxValue.
									DEC_ENABLE_PIN, 														// theEnablePin.
									DEC_CFG1_PIN,															// thecfg1Pin.
									DEC_CFG2_PIN, 															// thecfg2Pin.
									DEC_STEP_PIN, 															// theStepPin.			
									DEC_DIR_PIN, 															// theDirPin.
							 		1000000,																// theStepPeriod.
									20,																		// thePulsePeriod.
									1,																		// theHardwareTimer.
									&myDECMotor_ISR,														// theISR. 
									80, 																	// thePrescaler.
									10, 																	// theA.
									10, 																	// theD.
									0, 																		// theVs.
									DEFAULT_DEC_VM,															// theVm.
									200																		// theK.
									);

// Instancio objeto de pila de comandos.
ToteCommandFIFO myCommandStack = ToteCommandFIFO();

// Creamos el objeto de control del telescopio.
ToteTelesCtrlESPNOW myTeles = ToteTelesCtrlESPNOW();

void IRAM_ATTR myARMotor_ISR(void) {
	// Primero se comprueba si ya se está procesando la interrupción.
  	if (myARMotor.isHandlingISR() == true) {
		return;
	}

	// Compruebo final del movimiento.
	if (myARMotor.getIsFinished()) {
		// Se acabó.
		return;
	}

	// Indico que se está atendiendo la interrupción.
  	myARMotor.setHandlingISR(true);
	
	// Compruebo si es el flanco de subida.
	if (myARMotor.getEdge() == ToteESP32MotorTimer::RISING_EDGE) {
		myARMotor.incStepCount();  // Incremento la cuenta de pasos.

		digitalWrite(myARMotor.getStepPin(), HIGH); // Flanco de subida del pulso de activación de pasos.

		// Cambio el semáforo de flanco.
		myARMotor.setEdge(ToteESP32MotorTimer::FAILING_EDGE); // El flanco de bajada se producirá en...

		// Cambio la cuenta para el disparo del próximo flanco de bajada.
		myARMotor.setFailingEdgeCount();

	} else { // Si no es el de subida, estamos procesando el de bajada.
		// Flanco de bajada del pulso de activación de pasos.
		digitalWrite(myARMotor.getStepPin(), LOW);

		// Cambio el semáforo de flanco.
		myARMotor.setEdge(ToteESP32MotorTimer::RISING_EDGE); // El flanco de subida se producirá en...

		// Cambio la cuenta para el disparo del próximo flanco de subida.
		myARMotor.setRisingEdgeCount();
	}

  	// Nos preparamos para otra interrupción.
  	myARMotor.setHandlingISR(false);
}

void IRAM_ATTR myDECMotor_ISR(void) {
	// Primero se comprueba si ya se está procesando la interrupción.
  	if (myDECMotor.isHandlingISR() == true) {
		return;
	}

	// Compruebo final del movimiento.
	if (myDECMotor.getIsFinished()) {
		// Se acabó.
		return;
	}

	// Indico que se está atendiendo la interrupción.
  	myDECMotor.setHandlingISR(true);
	
	// Compruebo si es el flanco de subida.
	if (myDECMotor.getEdge() == ToteESP32MotorTimer::RISING_EDGE) {
		myDECMotor.incStepCount();  // Incremento la cuenta de pasos.

		digitalWrite(myDECMotor.getStepPin(), HIGH); // Flanco de subida del pulso de activación de pasos.

		// Cambio el semáforo de flanco.
		myDECMotor.setEdge(ToteESP32MotorTimer::FAILING_EDGE); // El flanco de bajada se producirá en...

		// Cambio la cuenta para el disparo del próximo flanco de bajada.
		myDECMotor.setFailingEdgeCount();

	} else { // Si no es el de subida, estamos procesando el de bajada.
		// Flanco de bajada del pulso de activación de pasos.
		digitalWrite(myDECMotor.getStepPin(), LOW);

		// Cambio el semáforo de flanco.
		myDECMotor.setEdge(ToteESP32MotorTimer::RISING_EDGE); // El flanco de subida se producirá en...

		// Cambio la cuenta para el disparo del próximo flanco de subida.
		myDECMotor.setRisingEdgeCount();
	}

  	// Nos preparamos para otra interrupción.
  	myDECMotor.setHandlingISR(false);
}

//==============
void setup() {
    Serial.begin(115200);

	_GOTO_SLAVE_DEBUG_(F("setup()"), F("configurando setup del esclavo."));

	// Configuro el pin del laser como salida.
	pinMode(LASER_PIN, OUTPUT);
	digitalWrite(LASER_PIN, LOW); 

	_GOTO_SLAVE_DEBUG_VALUE_(F("setup()"), F("ID Fabricante INA = "), myINA3221.getManufID());
  

    // Inicializo la comunicación ESP-NOW.
    myTeles.init();

    // Compruebo resultado
    if (myTeles.getESPNOWStatus()) {
        _GOTO_SLAVE_DEBUG_(F("setup()"), F("'getESPNOWStatus' devuelve 'true'."));
    } else {
        _GOTO_SLAVE_DEBUG_(F("setup()"), F("ERROR FATAL. 'getESPNOWStatus' devuelve 'false'."));
    }

	// Los pines los configura como salida el constructor del objeto TMCStepper. 
	// También se encarga de poner los valores lógicos apropiados para el inicio.
  
  	myARMotor.init();                   // Inicializamos el objeto del motor de Ascención Recta.
	myARMotor.setDriverEnabled(false);  // Deshabilito el driver.

	myDECMotor.init();                  // Inicializamos el objeto del motor de Declinación.
	myDECMotor.setDriverEnabled(false); // Deshabilito el driver.

	_GOTO_SLAVE_DEBUG_(F("setup()"), F("Inicializando INA3221"));
	pinMode(21, PULLUP);
	pinMode(22, PULLUP);
	myINA3221.begin();
}


void loop() {
    // Compruebo si hay mensajes por atender.
    if (myTeles.checkNewMsg()) {
		processMsg();
	}

    // Procesa comandos que estan en la pila.
	peekCommands();

	// Sondea INA3221.
	senseINA3221();
}

// Procesa los mensajes recibidos desde el CONTROLLER.
void processMsg(void) {
	// OJO. 'sendingDataPtr' es una puntero 'extern' a una estructura declarada 'ToteTelesCtrlESPNOW_VX_X.cpp'
	switch(receivedDataPtr->cmdRes) {
		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_HEARTBEAT_REQUEST:  // Se pide solicitud de latido. Respondo con el ACK
        	// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_HEARTBEAT_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
        break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_LASER_ON:	// Se pide encender el láser.
			_GOTO_SLAVE_DEBUG_(F("processMsg()"), F("Encendiendo Láser."));

			// Ponemos el pin a HIGH. 
			digitalWrite(LASER_PIN, HIGH);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_LASER_ON_ACK;

            // Enviamos mensaje.
            myTeles.sendData();		
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_LASER_OFF:	// Se pide apagar el láser.
			_GOTO_SLAVE_DEBUG_(F("processMsg()"), F("Apagando Láser."));
			
			// Ponemos el pin a LOW
			digitalWrite(LASER_PIN, LOW);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_LASER_OFF_ACK;

            // Enviamos mensaje.
            myTeles.sendData();		
		break;
		
		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_AR_MOTOR_DRIVER:
			// Este comando no pasa por la pila de comandos porque quiero que se ejecute inmediatamente.						
			// Habilito driver.
			myARMotor.setDriverEnabled(true);
						
			// Esta acción deshabilita la sincronización de AR.
			myARMotor.setSynced(false);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_AR_MOTOR_DRIVER_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_AR_MOTOR_DRIVER:
			// Este comando no pasa por la pila de comandos porque quiero que se ejecute inmediatamente.						
			// DEShabilito driver.
			myARMotor.setDriverEnabled(false);
						
			// Esta acción deshabilita la sincronización de AR.
			myARMotor.setSynced(false);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_AR_MOTOR_DRIVER_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_DEC_MOTOR_DRIVER:
			// Este comando no pasa por la pila de comandos porque quiero que se ejecute inmediatamente.						
			// Habilito driver.
			myDECMotor.setDriverEnabled(true);
						
			// Esta acción deshabilita la sincronización de AR.
			myDECMotor.setSynced(false);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_DEC_MOTOR_DRIVER_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_DEC_MOTOR_DRIVER:
			// Este comando no pasa por la pila de comandos porque quiero que se ejecute inmediatamente.						
			// DEShabilito driver.
			myDECMotor.setDriverEnabled(false);
						
			// Esta acción deshabilita la sincronización de AR.
			myDECMotor.setSynced(false);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_DEC_MOTOR_DRIVER_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_AR_DEC_MOTOR_DRIVERS:
			// Este comando no pasa por la pila de comandos porque quiero que se ejecute inmediatamente.						
			// Habilito driver.
			myARMotor.setDriverEnabled(true);
			myDECMotor.setDriverEnabled(true);
						
			// Esta acción deshabilita la sincronización de AR y DEC.
			myARMotor.setSynced(false);
			myDECMotor.setSynced(false);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_AR_DEC_MOTOR_DRIVERS_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_AR_DEC_MOTOR_DRIVERS:
			// Este comando no pasa por la pila de comandos porque quiero que se ejecute inmediatamente.						
			// Deshabilito driver.
			myARMotor.setDriverEnabled(false);
			myDECMotor.setDriverEnabled(false);
						
			// Esta acción deshabilita la sincronización de AR y DEC.
			myARMotor.setSynced(false);
			myDECMotor.setSynced(false);

			// Cargamos la estructura con la información a enviar.
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_AR_DEC_MOTOR_DRIVERS_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_STOP:
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::AR_MOTOR_STOP) {
				myCommandStack.push(ExecuteCommandEnum::AR_MOTOR_STOP);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_STOP_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));
				
				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_STOP:
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::DEC_MOTOR_STOP) {
				myCommandStack.push(ExecuteCommandEnum::DEC_MOTOR_STOP);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_STOP_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));
				
				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_STOP:
			// Paramos los motores.
			myARMotor.stop();
			myDECMotor.stop();

			// Vaciamos la pila de comandos.
			myCommandStack.reset();

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_STOP_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_HALT:
			// Deshabilitamos drivers.
			myARMotor.halt();
			myDECMotor.halt();

			// Limpiamos pila.
			myCommandStack.reset();

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_HALT_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTORS_NEXT_ROSTER_VM:
			// Agrego comando a la pila (Se puede repetir)
			myCommandStack.push(ExecuteCommandEnum::AR_DEC_MOTORS_NEXT_ROSTER_VM);

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTORS_NEXT_ROSTER_VM_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP:	
			// Agrego comando a la pila (solo si es distinto al último. Con esto se evita la repetición de comandos.)
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP) {
				myCommandStack.push(ExecuteCommandEnum::AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));
				
				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP:	
			// Agrego comando a la pila (solo si es distinto al último. Con esto se evita la repetición de comandos.)
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP) {
				myCommandStack.push(ExecuteCommandEnum::AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));
				
				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP:
			// Agrego comando a la pila solo si es distinto al último. Con esto se evita la repetición de comandos.
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP) {
				myCommandStack.push(ExecuteCommandEnum::DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));

				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP:
			// Agrego comando a la pila solo si es distinto al último. Con esto se evita la repetición de comandos.
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP) {
				myCommandStack.push(ExecuteCommandEnum::DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));

				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

			// Enviamos mensaje.
            myTeles.sendData();

		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_RESET_MOTOR_COUNTERS:
			// Esta acción desactiva la sincronización.
			myARMotor.setSynced(false);
			myDECMotor.setSynced(false);

			// Reseteamos contadores.
			myARMotor.resetStepCounters();
			myDECMotor.resetStepCounters();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_REQUEST:
			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_REQUEST_ACK;

			// Cargo la estructura con los datos.
			sendingData.ra  = myARMotor.getARFromAbsoluteStep();
			sendingData.dec = myDECMotor.getDECFromAbsoluteStep();
			
			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_INA3221_SENSE:
			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_INA3221_SENSE_ACK;

			// Cargo la estructura con los datos leidos por el INA3221.
			sendingData.vBattery = ina3221Data[0].busVoltage; 	// Los tres canales leen la tensión de la batería, así que leo el 1.
			sendingData.AR_mA = ina3221Data[0].current_mA;		// En el canal 1 (idx 0) está la alimentación del motor de AR.
			sendingData.DEC_mA = ina3221Data[1].current_mA;		// En el canal 2 (idx 1) está la alimentación del motor de AR.
			sendingData.REST_mA = ina3221Data[2].current_mA;	// En el canal 3 (idx 2) está la alimentación del resto de la electrónica.

			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_AXIS_FULL_REVOLUTION:
			// Agrego comando a la pila solo si es distinto al último. Con esto se evita la repetición de comandos.
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::AR_AXIS_FULL_REVOLUTION) {
				myCommandStack.push(ExecuteCommandEnum::AR_AXIS_FULL_REVOLUTION);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_AXIS_FULL_REVOLUTION_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));

				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_AXIS_FULL_REVOLUTION:
			// Agrego comando a la pila solo si es distinto al último. Con esto se evita la repetición de comandos.
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::DEC_AXIS_FULL_REVOLUTION) {
				myCommandStack.push(ExecuteCommandEnum::DEC_AXIS_FULL_REVOLUTION);

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_AXIS_FULL_REVOLUTION_ACK;
			} else {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));

				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}

			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_TRACKING_ON_OFF:
			if (!myARMotor.getIsTracking()) {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Activando tracking."));

				// Agrego comando a la pila solo si es distinto al último. Con esto se evita la repetición de comandos.
				if (myCommandStack.peekPrevious() != ExecuteCommandEnum::AR_MOTOR_TRACKING) {
					myCommandStack.push(ExecuteCommandEnum::AR_MOTOR_TRACKING);

					// Cargamos la estructura con la información a enviar
            		sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_TRACKING_ON_OFF_ACK;
				} else {
					_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));

					// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            		sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
				}
			} else {
				// Estaba activo, detengo el motor.
				if (myCommandStack.peekPrevious() != ExecuteCommandEnum::AR_MOTOR_STOP) {
					myCommandStack.push(ExecuteCommandEnum::AR_MOTOR_STOP);

					// Cargamos la estructura con la información a enviar
            		sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_TRACKING_ON_OFF_ACK;
				} else {
					_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));

					// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            		sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
				}
			}

	        // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_GOTO_ORIGIN:	
			// Agrego comando a la pila solo si es distinto al último. Con esto se evita la repetición de comandos.
			if (myCommandStack.peekPrevious() != ExecuteCommandEnum::AR_DEC_MOTORS_GOTO_ORIGIN) {
					myCommandStack.push(ExecuteCommandEnum::AR_DEC_MOTORS_GOTO_ORIGIN);
			} else {
				_GOTO_SLAVE_DEBUG_(F("ProceeMenu()"), F("Repetición de comando. No se añadirá a la pila de comandos."));

				// No se ha podido ejecutar el comando. Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CMD_STACK_REPEATED_NO_ACTION;
			}
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_COMPUTE_ARIES_ON:	
			// Actualizo el semáforo.
			isComputeAries = true;

			myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_COMPUTE_ARIES_OFF:	
			// Actualizo el semáforo.
			isComputeAries = false;

			myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_SYNC:
			// Esta acción habilita la sincronización de AR.
			myARMotor.syncAbsoluteStep(receivedDataPtr->utc, receivedDataPtr->ra);

			_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Motor de ascención recta sincronizado con estrella de referencia."));

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_SYNC_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;


		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_SYNC:
			// Esta acción habilita la sincronización de DEC.
			myDECMotor.syncAbsoluteStep(receivedDataPtr->dec);
			
			_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Motor de DEC sincronizado con estrella de referencia."));

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_SYNC_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_SYNC:
			// Esta acción habilita la sincronización de AR y DEC.
			myARMotor.syncAbsoluteStep(receivedDataPtr->utc, receivedDataPtr->ra);
			myDECMotor.syncAbsoluteStep(receivedDataPtr->dec);

			// Habilito el tracking.
			myARMotor.doTracking();
		
			_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Motores de AR/DEC sincronizados con estrella de referencia."));

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_SYNC_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;


		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GOTO_OBJECT:
			// Compruebo si se está sincronizado en AR.
			if (myARMotor.getIsSynced() == false) {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Goto cancelado. Eje AR no sincronizado."));

				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GOTO_OBJECT_CANCEL_AR_NOT_SYNCED;		

				// Enviamos mensaje.
            	myTeles.sendData();
				
				// Salimos.
				break;
			}

			// Compruebo si se está sincronizado en DEC.
			if (myDECMotor.getIsSynced() == false) {
				_GOTO_SLAVE_DEBUG_(F("processMenu()"), F("Goto cancelado. Eje DEC no sincronizado."));
				
				// Cargamos la estructura con la información a enviar
            	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GOTO_OBJECT_CANCEL_DEC_NOT_SYNCED;		

				// Enviamos mensaje.
            	myTeles.sendData();
				
				// Salimos.
				break;
			}

			// Iniciamos el Goto y verificamos que se va a poder hacer.
			if (myARMotor.gotoObject(receivedDataPtr->utc, receivedDataPtr->ra, isComputeAries) && myDECMotor.gotoObject(receivedDataPtr->dec)){
				// Se va ha poder hacer el goto.
				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GOTO_OBJECT_ACK;
			} else {
				// Indico que no se pudo hacer el goto.
				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GOTO_OBJECT_CANCEL_DID_NOT_ACCOMPLISH;
			}		
			
			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CANCEL_GOTO_OBJECT:
			myCommandStack.push(ExecuteCommandEnum::AR_MOTOR_STOP);
			myCommandStack.push(ExecuteCommandEnum::DEC_MOTOR_STOP);

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CANCEL_GOTO_OBJECT_ACK;		

			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GET_STATUS_INFO:
		
			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GET_STATUS_INFO_DATA;	

			// Cargo en la estructura los datos de estado.
			//sendingData.ra =  TOTE, TODO.
			//sendingData.dec =.
    		sendingData.isARMotorRunning 		= myARMotor.getIsRunning();
			sendingData.isARMotorSynced	 		= myARMotor.getIsSynced();
			sendingData.ARDECMotorsVmArrayIndex = myARMotor.getMotorRosterVmIndex();
			sendingData.isARTracking			= myARMotor.getIsTracking();
			sendingData.isDECMotorRunning 		= myDECMotor.getIsRunning();
			sendingData.isDECMotorSynced		= myDECMotor.getIsSynced();
			

    		
			//sendingData.utc =           // Hora UTC.	// TODO

			// Enviamos mensaje.
            myTeles.sendData();
		break;

		case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_PRINT_REPORT: // Se pide que muestre contadores en su Serial. Respondo con el ACK.
			// Muestro contadores.
			printReport();

			// Cargamos la estructura con la información a enviar
            sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_PRINT_REPORT_ACK;

            // Enviamos mensaje.
            myTeles.sendData();
		break;
	}
}

// Sondea la pila FIFO en busca de comandos a ejecutar.
void peekCommands(void) {
	uint8_t command;
	bool result = false;

	// Sondeo commando.
	command = myCommandStack.peek();

	switch(command) {
		case AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP:
			// Acepto el comando solo si el motor no se está moviendo.
			if (!myARMotor.getIsRunning()) {
				// Iniciamos movimiento manual y capturo resultado.
				result = myARMotor.manualPositioning(
						ToteESP32MotorTimerTMC::directionEnum::CCW,
						ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_4);

					_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP = "), result);
				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();
			
					// Esta acción deshabilita la sincronización de AR.
					myARMotor.setSynced(false);
				}
			} else {
				// Fuerzo parada del motor. El loop seguirá sondeando hasta que se pueda ejecutar el comando.
				myARMotor.stop();
			}
		break;

		case AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP:
			// Acepto el comando solo si el motor no se está moviendo.
			if (!myARMotor.getIsRunning()) {
				// Iniciamos movimiento manual y capturo resultado.
				result = myARMotor.manualPositioning(
						ToteESP32MotorTimerTMC::directionEnum::CW,
						ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_4);
		
				_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP = "), result);

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();

					// Esta acción deshabilita la sincronización de AR.
					myARMotor.setSynced(false);
				}
			} else {
				// Fuerzo parada del motor. El loop seguirá sondeando hasta que se pueda ejecutar el comando.
				myARMotor.stop();
			}
		break;	

		case AR_MOTOR_STOP:
			// Acepto el comando solo si el motor SE ESTÁ moviendo.
			if (myARMotor.getIsRunning()) {
				// Compruebo si el movimiento era debido al tracking o no.
				if (!myARMotor.getIsTracking()) { // Si NO estaba haciendo tracking, entonces sí se pierde la sincronización.
					// Esta acción deshabilita la sincronización de AR.
					myARMotor.setSynced(false);
				}

				// Iniciamos movimiento manual y capturo resultado.
				// Deteniendo motor.
				result = myARMotor.stop();

				_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de AR_MOTOR_STOP = "), result);

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();
				}
			} else {
				// Si el motor está parado, entonces no se hace nada.
			}
		break;

		case DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP:
			// Acepto el comando solo si el motor no se está moviendo.
			if (!myDECMotor.getIsRunning()) {
				// Iniciamos movimiento manual y capturo resultado.
				result = myDECMotor.manualPositioning(
							ToteESP32MotorTimerTMC::directionEnum::CCW,
							ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_4);

				_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP = "), result);

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();

					// Esta acción deshabilita la sincronización de DEC.
					myDECMotor.setSynced(false);
				}
			} else {
				// Fuerzo parada del motor. El loop seguirá sondeando hasta que se pueda ejecutar el comando.
				myDECMotor.stop();
			}
		break;

		case DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP:
			// Acepto el comando solo si el motor no se está moviendo.
			if (!myDECMotor.getIsRunning()) {
			// Iniciamos movimiento manual y capturo resultado.
				result = myDECMotor.manualPositioning(
						ToteESP32MotorTimerTMC::directionEnum::CW,
						ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_4);

				_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP = "), result);

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();
					
					// Esta acción deshabilita la sincronización de DEC.
					myDECMotor.setSynced(false);
				}
			} else {
				// Fuerzo parada del motor. El loop seguirá sondeando hasta que se pueda ejecutar el comando.
				myDECMotor.stop();
			}
		break;

		case DEC_MOTOR_STOP:
			// Acepto el comando solo si el motor SE ESTÁ moviendo.
			if (myDECMotor.getIsRunning()) {
				// Iniciamos movimiento manual y capturo resultado.
				// Deteniendo motor.
				result = myDECMotor.stop();

				_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de DEC_MOTOR_STOP = "), result);

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();
			
					// Esta acción deshabilita la sincronización de DEC.
					myDECMotor.setSynced(false);
				}
			} else {
				// Si el motor está parado, entonces no se hace nada.
			}
		break;

		case AR_DEC_MOTORS_GOTO_ORIGIN:
			// Acepto el comando solo si los motores no se están moviendo.
			if (!myARMotor.getIsRunning() && !myDECMotor.getIsRunning()) {
				// Iniciamos movimiento manual y capturo resultado.
				result = myARMotor.gotoOrigin() & myDECMotor.gotoOrigin();

				_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de AR_DEC_MOTORS_GOTO_ORIGIN = "), result);

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();
			
					// Esta acción deshabilita la sincronización de AR y DEC
					myARMotor.setSynced(false);
					myDECMotor.setSynced(false);
				}
			} else {
				// Fuerzo parada de los motores. El loop seguirá sondeando hasta que se pueda ejecutar el comando.
				myARMotor.stop();
				myDECMotor.stop();
			}
		break;


		case AR_DEC_MOTORS_NEXT_ROSTER_VM:
			// Acepto el comando solo si los motores no se están moviendo.
			if (!myARMotor.getIsRunning() && !myDECMotor.getIsRunning()) {
				// Siguiente  velocidad. Solo es necesario ponerlo en un motor porque el miembro es estático y se comparte entre todas las instancias deo objeto.
				result = myARMotor.nextMotorRosterVm();

				// Pongo la velocidad en cada motor.
				myARMotor.setVm((ToteESP32MotorTimerTMC::motorVmEnum) myARMotor.getMotorRosterVmIndex()); 	// El casting es necesario para que se llame al método sobrecargado apropiado.
				myDECMotor.setVm((ToteESP32MotorTimerTMC::motorVmEnum) myDECMotor.getMotorRosterVmIndex());	// El casting es necesario para que se llame al método sobrecargado apropiado.

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();
				
					// Esta acción deshabilita la sincronización de AR y DEC.
					myARMotor.setSynced(false);
					myDECMotor.setSynced(false);
				}
			} else {
				// Fuerzo parada del motor. El loop seguirá sondeando hasta que se pueda ejecutar el comando.
				myARMotor.stop();
			}
		break;

		case AR_MOTOR_TRACKING:
			// Acepto el comando solo si el motor no se está moviendo.
			if (!myARMotor.getIsRunning()) {
				// Activo tracking.
				result = myARMotor.doTracking();
			
				_GOTO_SLAVE_DEBUG_VALUE_(F("peekCommands"), F("Resultado de AR_MOTOR_TRACKING = "), result);

				// Comprobación de resultado.
				if (result) {
					// Si el resultado es 'true', entonces retiro el comando de la pila.
					myCommandStack.pop();

					// Esta acción deshabilita la sincronización de AR porque no sé cuánto tiempo se lleva así y no es posible calcular Aries.
					myARMotor.setSynced(false);
				}
			} else {
				// Fuerzo parada del motor. El loop seguirá sondeando hasta que se pueda ejecutar el comando.
				myARMotor.stop();
			}
		break;

		case AR_AXIS_FULL_REVOLUTION:
			_GOTO_SLAVE_DEBUG_(F("peekCommands()"), F("Girando vuelta completa eje AR."));

			// Habilito driver.
			myARMotor.setDriverEnabled(true);

			// Esta acción deshabilita la sincronización de AR.
			myARMotor.setSynced(false);

			// Asumiendo 130 dientes.
			result = myARMotor.move(1248000, ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_4, ToteESP32MotorTimerTMC::accelConstantEnum::ACCELERATED, false);
			
			if (result) {
				// Si el resultado es 'true', entonces retiro el comando de la pila.
				myCommandStack.pop();

				// Esta acción deshabilita la sincronización de AR.
				myARMotor.setSynced(false);
			}
		break;

		case DEC_AXIS_FULL_REVOLUTION:
			_GOTO_SLAVE_DEBUG_(F("peekCommands()"), F("Girando vuelta completa eje DEC."));

			// Habilito driver.
			myDECMotor.setDriverEnabled(true);

			// Esta acción deshabilita la sincronización de AR.
			myDECMotor.setSynced(false);

			// Asumiendo 65 dientes.
			result = myDECMotor.move(624000, ToteESP32MotorTimerTMC::stepTypeLegacyEnum::TMC_2208_LEGACY_STEP_1_4, ToteESP32MotorTimerTMC::accelConstantEnum::ACCELERATED, false);				

			if (result) {
				// Si el resultado es 'true', entonces retiro el comando de la pila.
				myCommandStack.pop();

				// Esta acción deshabilita la sincronización de DEC.
				myDECMotor.setSynced(false);
			}
		break;
	}
}

// Sondeo el INA3221
void senseINA3221(void) {
	// Compruebo si es el momento.
	if (millis() - lastINASenseMillis > INA3221_SENSE_PERIOD) {		// Leo los canales del INA. El array va de 0 a 2, los canales de 1 a 3.

		for (int8_t i = 0; i <= 2; i++) {
			ina3221Data[i].busVoltage = myINA3221.getBusVoltage_V(i + 1);
			ina3221Data[i].current_mA = myINA3221.getCurrent_mA(i + 1) -2; // Quito 2mA de pruebas que he hecho de calibración. 
		}

		// Reiniciamos la cuenta.
		lastINASenseMillis = millis();
	}
}

void printReport(void) {
	_GOTO_SLAVE_DEBUG_(F("printReport()"), F("********************"));
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("AR MOTOR: Procesando paso "), (long) myARMotor.getStep());				
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("AR MOTOR: microStepRes = "), (long) myARMotor.getMicroStepRes());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("AR MOTOR: absoluteStep (CCW resta, CW suma) contados (en unidades de 1/microStepRes)= "), (long) myARMotor.getAbsoluteStep());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("AR MOTOR: isRunning = "), myARMotor.getIsRunning());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("AR MOTOR: isFinished = "), myARMotor.getIsFinished());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("AR MOTOR: isTracking = "), myARMotor.getIsTracking());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("DEC MOTOR: Procesando paso "), (long) myDECMotor.getStep());				
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("DEC MOTOR: microStepRes = "), (long) myDECMotor.getMicroStepRes());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("DEC MOTOR: absoluteStep (CCW resta, CW suma) contados (en unidades de 1/microStepRes)= "), (long) myDECMotor.getAbsoluteStep());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("DEC MOTOR: isRunning = "), myDECMotor.getIsRunning());
	_GOTO_SLAVE_DEBUG_VALUE_(F("printReport()"), F("DEC MOTOR: isFinished = "), myDECMotor.getIsFinished());

	for (int8_t i = 0; i <= 2; i++) {
		_GOTO_SLAVE_DEBUG_VALUE_(F("senseINA3221()"), F("Canal = "), i + 1);
		_GOTO_SLAVE_DEBUG_VALUE_(F("senseINA3221()"), F("V = "), ina3221Data[i].busVoltage);
		_GOTO_SLAVE_DEBUG_VALUE_DOUBLE_(F("senseINA3221()"), F("mA = "), ina3221Data[i].current_mA);
	}	
}
