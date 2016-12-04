/* webturtle - An ESP8266 version of the Arduino Drawing Robot
(http://www.instructables.com/id/Arduino-Drawing-Robot/) allowing
control via WiFi and storage of logo procedures in its own (SPIFFS)
file system.

The program is derived from
- MakersBox's TIRL_Arduino_TEST.zip found in the instructable
- Hristo Gochkov's FSWebServer (https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WebServer/examples/FSBrowser/FSBrowser.ino)
- A code contribution made to the instructable by PMPU_student

All copyrights and licenses from the derived code applies to this program
*/
#include <stdlib.h>
#include <string.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <Servo.h>

File fsUploadFile;
File fin;
File fcmd;
File fproc;
File fcur;


char procname[16];
int argcnt;

#undef TURTLE_DEBUG
typedef void (* fp) (float);
typedef enum { INTEGER, REAL, VARID, PROCID, CMD } Type;
typedef struct {
  Type type;
  union {
    int integer;
    float real;
    int varid;
    int procid;
    int cmd;
    uint8_t breal[4];
  };
} Value;

const char* ssid = "Turtle";
const char* password = "LogoTurtle";
const char* host = "turtle";

// setup servo
int servoPin = 16;
int PEN_DOWN = 170; // angle of servo when pen is down
int PEN_UP = 80;   // angle of servo when pen is up
Servo penServo;

int wheel_dia=63; //      # mm (increase = spiral out) was 66.25 63
int wheel_base=114.25; //,    # mm (increase = spiral in) was 112 114
int steps_rev=128; //,     # 512 for 64x gearbox, 128 for 16x gearbox
int delay_time=6; //            # time between steps in ms

// Stepper sequence org->pink->blue->yel
int L_stepper_pins[] = {5, 0, 2, 4};
int R_stepper_pins[] = {15, 12, 14, 13};

int fwd_mask[][4] =  {{1, 0, 1, 0},
                      {0, 1, 1, 0},
                      {0, 1, 0, 1},
                      {1, 0, 0, 1}};

int rev_mask[][4] =  {{1, 0, 0, 1},
                      {0, 1, 0, 1},
                      {0, 1, 1, 0},
                      {1, 0, 1, 0}};

ESP8266WebServer server(80);
String form = "<!DOCTYPE html><html><body><p>Command the Turtle</p><form action='turtle' method='post'><br><textarea name='tcom' rows='10' cols='30'></textarea><br><input type='submit' value='Submit'></form>";

#define INPUT_BUFFER_LENGTH 500
char input_buffer[INPUT_BUFFER_LENGTH];
int inswitch;

#define TOKEN_BUFFER_LENGTH 500
char token_buffer[TOKEN_BUFFER_LENGTH];

void forward(float distance);
void backward(float distance);
void right(float degrees);
void left(float degrees);
void penup(float dum);
void pendown(float dum);
void done(float dum);

#define CMD_CNT 13
struct cmdstruct {
  char * cmdname[CMD_CNT] = {"FD", "BK", "RT", "LT", "PU", "PD", "RPT", "[", "]", "DONE", "TO", "END", "FILE"};
  int tokentype[CMD_CNT] = {1, 1, 1, 1, 0, 0, 3, 0, 0, 0, 2, 0, 7};
  fp cmdfunc[CMD_CNT] = {forward, backward, right, left, penup, pendown, NULL, NULL, NULL, done, NULL, NULL, NULL};
};
cmdstruct cmdlist;

#define SYM_NAME_LENGTH 24
struct symstruct {
  char sym_name[SYM_NAME_LENGTH];
  Value sym_value;
};
#define SYM_TABLE_LENGTH 100
symstruct symtable[SYM_TABLE_LENGTH]; // The ID of a variable is its location in this buffer
int symcnt; // Always points to the last used location in the symbol table (unless -1 = empty)

int state;
int proc; // Used by processInput
char *current_token; // Used by processInput and getParam

struct pgmstruct {
  char fname[24]; // file  being read
  size_t fpos; // position in file
  int rptcount; // state of repeat count
  size_t rptstrt; // state of start of repeat commands
};

#define MAXHEAPSIZE 100
pgmstruct heap[MAXHEAPSIZE];
int heapptr = 0;
#define hpush(h) heap[heapptr++] = h
#define hpop heap[--heapptr]

pgmstruct active;

int step(float distance){
  int steps = distance * steps_rev / (wheel_dia * 3.1412); //24.61
#ifdef TURTLE_DEBUG
    Serial.print(distance);
    Serial.print(" ");
    Serial.print(steps_rev);
    Serial.print(" ");  
    Serial.print(wheel_dia);
    Serial.print(" ");  
    Serial.println(steps);
    delay(1000);
#endif
  return steps;  
}

void forward(float distance){
  int steps = step(distance);
#ifdef TURTLE_DEBUG
  Serial.print("FORWARD()");
  Serial.println(distance);
#endif
  for(int step=0; step<steps; step++){
    for(int mask=0; mask<4; mask++){
      for(int pin=0; pin<4; pin++){
        digitalWrite(L_stepper_pins[pin], rev_mask[mask][pin]);
        digitalWrite(R_stepper_pins[pin], fwd_mask[mask][pin]);
      }
      delay(delay_time);
    } 
  }
}

void backward(float distance){
  int steps = step(distance);
#ifdef TURTLE_DEBUG
  Serial.print("BACKWARD()");
  Serial.println(distance);
#endif
  for(int step=0; step<steps; step++){
    for(int mask=0; mask<4; mask++){
      for(int pin=0; pin<4; pin++){
        digitalWrite(L_stepper_pins[pin], fwd_mask[mask][pin]);
        digitalWrite(R_stepper_pins[pin], rev_mask[mask][pin]);
      }
      delay(delay_time);
    } 
  }
}

float getNearestAngle(float angle_){
  float angle = 0;
  int step = 0;
  float previousAngle = 0;
  float step_length = 3.1412 * wheel_dia / steps_rev;
  while(!(previousAngle <= angle_ && angle_ <= angle)){
    step += 1;
    previousAngle = angle;
    angle = step * step_length * 360 / (wheel_base * 3.1412) + 0.01;
  }
  float dif1 = angle_- angle;
  float dif2 = angle_- previousAngle;
  if(abs(dif1) < abs(dif2)){
    return angle;
  }else{
    return previousAngle;
  }
}

void right(float degrees){
  float rotation = getNearestAngle(degrees) / 360.0;
  float distance = wheel_base * 3.1412 * rotation;
  int steps = step(distance);
#ifdef TURTLE_DEBUG1
  Serial.print("RIGHT()");
  Serial.println(degrees);
#endif
  for(int step=0; step<steps; step++){
    for(int mask=0; mask<4; mask++){
      for(int pin=0; pin<4; pin++){
        digitalWrite(R_stepper_pins[pin], rev_mask[mask][pin]);
        digitalWrite(L_stepper_pins[pin], rev_mask[mask][pin]);
      }
      delay(delay_time);
    } 
  }   
}

void left(float degrees){
  float rotation = getNearestAngle(degrees) / 360.0;
  float distance = wheel_base * 3.1412 * rotation;
  int steps = step(distance);
#ifdef TURTLE_DEBUG1
  Serial.print("LEFT()");
  Serial.println(degrees);
#endif
  for(int step=0; step<steps; step++){
    for(int mask=0; mask<4; mask++){
      for(int pin=0; pin<4; pin++){
        digitalWrite(R_stepper_pins[pin], fwd_mask[mask][pin]);
        digitalWrite(L_stepper_pins[pin], fwd_mask[mask][pin]);
      }
      delay(delay_time);
    } 
  }   
}

void done(float dum){ // unlock stepper to save battery
#ifdef TURTLE_DEBUG
  Serial.println("DONE()");
#endif
  for(int mask=0; mask<4; mask++){
    for(int pin=0; pin<4; pin++){
      digitalWrite(R_stepper_pins[pin], LOW);
      digitalWrite(L_stepper_pins[pin], LOW);
    }
    delay(delay_time);
  }
}

void penup(float dum){
  delay(250);
#ifdef TURTLE_DEBUG
  Serial.println("PEN_UP()");
#endif
  penServo.write(PEN_UP);
  delay(250);
}

void pendown(float dum){
  delay(250);  
#ifdef TURTLE_DEBUG
  Serial.println("PEN_DOWN()");
#endif
  penServo.write(PEN_DOWN);
  delay(250);
}

String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
#ifdef TURTLE_DEBUG
  Serial.println("handleFileRead: " + path);
#endif
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload(){
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
#ifdef TURTLE_DEBUG
    Serial.print("handleFileUpload Name: "); Serial.println(filename);
#endif
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
//    Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
  }
}

void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
#ifdef TURTLE_DEBUG
  Serial.println("handleFileDelete: " + path);
#endif
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
#ifdef TURTLE_DEBUG
  Serial.println("handleFileCreate: " + path);
#endif
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
#ifdef TURTLE_DEBUG
  Serial.println("handleFileList: " + path);
#endif
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}

void getParam(Type t) {
  int i;
  int j = -1;
  char fqvn[SYM_NAME_LENGTH];
  if(current_token[0] == ':') { // It's a variable; store the type and its ID
    if(fproc != NULL) {
      strcpy(fqvn, procname);
      strcat(fqvn, current_token);
      for(i = 0; i <= symcnt; i++) {
        if(!strcmp(symtable[i].sym_name, fqvn)) j = i;
      }
      fproc.write((uint8_t) VARID);
      fproc.write((uint8_t) j);
    }
    else {
      if(fcmd != NULL) {
        for(i = 0; i <= symcnt; i++) {
          if(!strcmp(symtable[i].sym_name, current_token)) j = i;
        }
        fcmd.write((uint8_t) VARID);
        fcmd.write((uint8_t) j);
      }
    }
  }
  else { // It's a constant; store the type and its value
    Value r;
    if(t == REAL){
      r.real = atof(current_token);
      if(fproc != NULL) {
        fproc.write((uint8_t) REAL);
        fproc.write(r.breal, 4);
      }
      else {
        if(fcmd != NULL) {
          fcmd.write((uint8_t) REAL);
          fcmd.write(r.breal, 4);
        }
      }
    }
    else {
      r.real = atoi(current_token);
      if(fproc != NULL) {
        fproc.write((uint8_t) INTEGER);
        fproc.write(r.breal, 2);
      }
      else {
        if(fcmd != NULL) {
          fcmd.write((uint8_t) INTEGER);
          fcmd.write(r.breal, 2);
        }
      }
    }
  }
}

char* getNextToken(char* buffer) {
  if(inswitch == 0) { // we're reading from input_buffer
    if(buffer != NULL) return(strtok(buffer," \r\n"));
    else return(strtok(NULL," \r\n"));
  }
  else { // Read from the file
    int done = 0;
    int cnt = 0; // always points to next available buffer location
    char inpch;
    while(done == 0) {
      if(fin.available()) {
        inpch = fin.read();
        if((inpch == ' ') || (inpch == '\r') || (inpch == '\n')) { // check for white characters
          if(cnt > 0) { // we have a non-blank string
            token_buffer[cnt] = NULL; // Terminate it
            done = 1;
          } // (Otherwise, we're trimming white characters from the beginning of the string)
        }
        else token_buffer[cnt++] = inpch;
      }
      else { // EOF. Terminate the string
        token_buffer[cnt] = NULL;
        fin.close();
        done = 1;
      }
    }
    return(token_buffer);
  }
}

void processInput(char* buffer) {
  state =0;
  proc = 0;
  int i,x;
  int replay = 0;
  int found;
  argcnt = 0;
  symcnt = -1;
  fproc = (fs::File) NULL;
  fcmd = SPIFFS.open("/cmd", "w");
  if (!fcmd) {
    Serial.println("file open failed");
  }
  current_token = getNextToken(buffer);
  while (current_token[0] != NULL){
#ifdef TURTLE_DEBUG
    Serial.print("current_token = ");
    Serial.print(current_token);
    Serial.print(" state = ");
    Serial.println(state);
#endif
    switch (state) {
      case 0: // get the next command
        found = 0;
        for(i=0; i<CMD_CNT; i++){
          if(!strcmp(current_token, cmdlist.cmdname[i])){ // It's a command, but where does it go?
            if((i != 10) && (i != 12)) { // we'll store it if its not the TO command or the FILE command
              if(fproc != NULL) { // goes to proc currently being defined
                if(i != 11) { // as long as it's not the END command
                  fproc.write((uint8_t) CMD);
                  fproc.write((uint8_t) i);
                }
                else { // It is, so close the proc file
                  fproc.close();
                  fproc = (fs::File) NULL;
                }
              }
              else { // goes to command file
                fcmd.write((uint8_t) CMD);
                fcmd.write((uint8_t) i);                
              }
            }
            state = cmdlist.tokentype[i];
            found = 1;
          }
        }
        if(found == 0){ // Check to see if it's a procedure call
          // TODO: Make procedure calls recursive
          for(i = 0; i <= symcnt; i++) {
            if(!strcmp(symtable[i].sym_name, current_token)) {
              if(fproc != NULL){
                fproc.write((uint8_t) PROCID);
                fproc.write((uint8_t) i); // store the sym table entry number
              }
              else {
                fcmd.write((uint8_t) PROCID);
                fcmd.write((uint8_t) i); // store the sym table entry number
              }
              argcnt = symtable[i].sym_value.integer;
              if(argcnt > 0) state = 6; // we need to process its argument list
#ifdef TURTLE_DEBUG
              else Serial.println("No arguments to process");
#endif
              found = 1;
            }
          }
          if(found == 0) {
            Serial.print("Unknown command: ");
            Serial.println(current_token);
          }
        }
        break;
      case 1: // motion cmd - get  one argument
        getParam(REAL);
        state = 0;
        break;
      case 2: // Previous token was TO; this token should be the name of the procedure
        char nproc[SYM_NAME_LENGTH];
        strcpy(nproc, "/p/");
        strcat(nproc, current_token);
        fproc = SPIFFS.open(nproc, "w");
        if (!fproc) {
          Serial.println("file open failed");
        }
#ifdef TURTLE_DEBUG
        Serial.print("Writing to ");
        Serial.println(nproc);
#endif
        strcpy(procname, current_token);
        strcpy(symtable[++symcnt].sym_name, current_token); // add it to the symbol table
        symtable[symcnt].sym_value.type = PROCID;
        symtable[symcnt].sym_value.integer = 0; // Placeholder for total number of arguments in procedure
        
        proc = symcnt; // hold onto this location for now; needed for state 5
        state = 5;
        break;
      case 3: // Repeat cmd - get repeat count
        getParam(INTEGER);
        state++;
        break;
      case 4: // Currently not used
        state = 0;
        break;
      case 5: // get argument list for TO command
        if(current_token[0] == ':') { // Has to be a variable
          symtable[++symcnt].sym_value.type = VARID;
          strcpy(symtable[symcnt].sym_name, procname); // prefix var with proc name
          strcat(symtable[symcnt].sym_name, current_token); // add it to the symbol table
          symtable[symcnt].sym_value.real = 0.0; // assume it's a float and initialize it
          fproc.write((uint8_t) VARID);
          fproc.write((uint8_t) symcnt);
          // Note that, unlike above, we're not storing the type with the argument          
          symtable[proc].sym_value.integer++; // increase argument count
        }
        else { // This token isn't an argument; send it back through        
          replay = 1;
          state = 0; // Start collecting the commands that comprise the procedure
        }
#ifdef TURTLE_DEBUG
        if(symtable[proc].sym_value.integer == 0) Serial.println("This proc has no arguments");
#endif
        break;
      case 6: // Process the arguments of a procedure call
        if(argcnt-- > 0) {
          getParam(REAL);
        }
        else { // This isn't a calling argument; send it back through
          replay = 1;
          state = 0;
        }
        break;
      case 7: // Get the file name to use as input
        fin = SPIFFS.open(current_token, "r");
        if (!fin) {
          Serial.println("file open failed");
        }
        inswitch = 1;
        state = 0;
        break;
    default:
        state = 0;
        break;
    }
    delay(1); // give the network stack a chance
    if(replay == 1) replay = 0; // we're going to re-process the last token
    else current_token = getNextToken(NULL); // get next token
  }
  fcmd.write((uint8_t) CMD);
  fcmd.write((uint8_t) 9);                
  fcmd.close();
 /* fcmd = SPIFFS.open("/cmd", "r");
  if (!fcmd) {
    Serial.println("/cmd file open failed for reading");
  }
  x=0;
  for(i=0;i<fcmd.size();i++){
    Serial.print(fcmd.read(), HEX);
    Serial.print(" ");
    if(x++ > 30) {
      Serial.println(".");
      x = 0;
    }
  }
  fcmd.close();
  Serial.println(".");
  fcmd = SPIFFS.open("/p/path3341", "r");
  if (!fcmd) {
    Serial.println("/p/path3341 file open failed for reading");
  }
  Serial.print("/p/path3341 file size = ");
  Serial.println(fcmd.size());
  x=0;
  for(i=0;i<fcmd.size();i++){
    Serial.print(fcmd.read(), HEX);
    Serial.print(" ");
    if(x++ > 30) {
      Serial.println(".");
      x = 0;
    }
  }
  fcmd.close();
  Serial.println("."); */
}

void interpretInput() {
  uint8_t i,j,k, a;
  int b,c;
  int running, pgmlevel;
  Value cur_tok;
  char nproc[SYM_NAME_LENGTH];
  strcpy(active.fname, "/cmd");
  active.fpos = 0;
  active.rptcount = 0;
  active.rptstrt = 0;

#ifdef TURTLE_DEBUG
          fcmd = SPIFFS.open(active.fname, "r");
          if (!fcmd) {
            Serial.print(active.fname);
            Serial.println("file open failed for reading");
          }
          Serial.print("file size = ");
          Serial.println(fcmd.size());
          b=0;
          for(c=0;c<fcmd.size();c++){
            fcmd.read(&a, 1);
            Serial.print(a, HEX);
            Serial.print(" ");
            if(b++ > 30) {
              Serial.println(".");
              b = 0;
            }
          }
          fcmd.close();
          Serial.println(". End of dump");
#endif          
  fcur = SPIFFS.open(active.fname, "r");
  if (!fcur) {
    Serial.print(active.fname);
    Serial.println("file open failed for reading");
  }        
  pgmlevel = 0;
  running = 1;
  uint8_t ttype;
  while(running) {
    if(fcur.available()) {
      fcur.read(&ttype, 1);
#ifdef TURTLE_DEBUG
      Serial.print("Token type = ");
      Serial.println(ttype);
#endif
      switch(ttype) {
        case CMD:
          fcur.read(&i, 1);
#ifdef TURTLE_DEBUG
          Serial.print("CMD name = ");
          Serial.println(cmdlist.cmdname[i]);
#endif
          switch(i) { // This will be the command type
            case 0: // Forward
            case 1: // Backward
            case 2: // Right
            case 3: // Left
              fcur.read(&j, 1);
              if(j == REAL) { // It's a constant
                fcur.read(cur_tok.breal, 4);
                cmdlist.cmdfunc[i](cur_tok.real);
              }
              else {
                if(j == VARID) { // It's a variable
                  fcur.read(&k, 1);
                  cmdlist.cmdfunc[i](symtable[k].sym_value.real);
                }
              }
              break;
            case 4: // Penup
            case 5: // Pendown
            case 9: // Done
              cmdlist.cmdfunc[i](0.0);
              break;
            case 6: // Repeat
              fcur.read(&j, 1); // get repeat count
              if(j == INTEGER) { // It's a constant
                fcur.read(cur_tok.breal, 2);
                active.rptcount = cur_tok.integer;
              }
              else {
                if(j == VARID) { // It's a variable
                  fcur.read(&k, 1);
                  active.rptcount = symtable[k].sym_value.integer;
                }
              }
              break;
            case 7: // Start of repeat commands ("[")
              active.rptstrt = fcur.position(); // Repeat commands start at this file position
              break;
            case 8: // End of repeat commands ("]")
              if(--active.rptcount > 0) { // Let's do it again
                fcur.seek(active.rptstrt, SeekSet); // position file back to start of repeat commands
              }
              break;
          } // switch i
          break;
        case PROCID:
          fcur.read(&i, 1); // get the proc's symtable entry number
          argcnt = symtable[i].sym_value.integer; // the proc's number of arguments
#ifdef TURTLE_DEBUG
          Serial.print("Procedure ");
          Serial.print(symtable[i].sym_name);
          Serial.print(" Number of arguments = ");
          Serial.println(argcnt);
#endif
          strcpy(nproc, "/p/");
          strcat(nproc, symtable[i].sym_name); // name of proc file
#ifdef TURTLE_DEBUG
          fcmd = SPIFFS.open(nproc, "r");
          if (!fcmd) {
            Serial.println(nproc);
            Serial.println("file open failed for reading");
          }
          Serial.print("file size = ");
          Serial.println(fcmd.size());
          b=0;
          for(c=0;c<fcmd.size();c++){
            fcmd.read(&a, 1);
            Serial.print(a, HEX);
            Serial.print(" ");
            if(b++ > 30) {
              Serial.println(".");
              b = 0;
            }
          }
          fcmd.close();
          Serial.println(". End of dump");
#endif          
          fproc = SPIFFS.open(nproc, "r");  // We need to get the variable IDs of the arguments from this file
          if (!fproc) {
            Serial.print("file open failed for reading ");
            Serial.println(nproc);
          }
          if(argcnt > 0) {
            while(argcnt-- > 0) { // initialize the values of the arguments
              fproc.read(&k, 1); // Should be VARID
              fproc.read(&k, 1); // Here's the argument's ID in the symbol table
              fcur.read(&j, 1);
              if(j == REAL) { // It's a constant
                fcur.read(cur_tok.breal, 4);
                symtable[k].sym_value.real = cur_tok.real; // update the argument's value
              }
              else {
                if(j == VARID) { // It's a variable
                  fcur.read(&j, 1);
                  symtable[k].sym_value.real = symtable[j].sym_value.real;
                }
              }
            }
          }
          // Save the current state of the program
          active.fpos = fcur.position();
#ifdef TURTLE_DEBUG
          Serial.print("Saved last file position = ");
          Serial.println(active.fpos);
#endif
          hpush(active);
          fcur.close();
          strcpy(active.fname, nproc);
          active.fpos = fproc.position();
          fproc.close();
          fcur = SPIFFS.open(active.fname, "r");
          if (!fcur) {
            Serial.print("file open failed for reading ");
            Serial.println(active.fname);
          }
          fcur.seek(active.fpos, SeekSet); // position file
#ifdef TURTLE_DEBUG
          Serial.print("Initial position of this file = ");
          Serial.println(active.fpos);
#endif
          active.rptcount = 0;
          active.rptstrt = 0;
          pgmlevel++;
          break;
        default:
          break;
      } // switch fcur
    } // available
    else { // ran through all the tokens in this file
      fcur.close();
      if(--pgmlevel < 0) running = 0; // That's the end of the command buffer -- exit the interpreter
      else { // We were in a procedure -- restore previous state
        active = hpop;
        fcur = SPIFFS.open(active.fname, "r");
        if (!fcur) {
          Serial.print("file open failed for reading ");
          Serial.println(active.fname);
        }
        fcur.seek(active.fpos, SeekSet); // position file
#ifdef TURTLE_DEBUG
        Serial.print("Restoring file position = ");
        Serial.println(active.fpos);
#endif
      }
    }
  } // while
}

void handle_index() {
  server.send(200,"text/html", form);
}

// TODO: Generate the form dynamically to add a status field
void handle_form(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  //Print out the amount of arguments received in POST payload
#ifdef TURTLE_DEBUG
    Serial.println(server.args());
    for (int x=0; x < server.args(); x++) {
      //print all arguments received in POST payload
      Serial.print(server.argName(x));
      Serial.print(":");
      Serial.println(server.arg(x));
    }
#endif
  if(!strcmp(server.argName(0).c_str(),"tcom")) {
    strncpy(input_buffer, server.arg(0).c_str(), INPUT_BUFFER_LENGTH);
    for(int x = 0; x < strlen(input_buffer); x++) input_buffer[x] = toupper(input_buffer[x]);
    fin = (fs::File) NULL; // signifies that we'll be reading from input_buffer initially
    inswitch = 0;
    Serial.println("Begin Processing");
    processInput(input_buffer);
    interpretInput();
    Serial.println("End Processing");
  }
  else return server.send(500, "text/plain", "BAD FORM");
  //Make the browser happy, send him something
  server.send(200, "text/html", form);
}

/**
 *  Define our setup function and then loop
 **/
void setup(){
  
  Serial.begin(115200);           // set up Serial library at 11520 bps
  SPIFFS.begin();
  {
    Dir dir = SPIFFS.openDir("/");
#ifdef TURTLE_DEBUG
    while (dir.next()) {    
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    Serial.printf("\n");
#endif
  }
  //set up the steppers
   for(int pin=0; pin<4; pin++){
    pinMode(L_stepper_pins[pin], OUTPUT);
    digitalWrite(L_stepper_pins[pin], LOW);
    pinMode(R_stepper_pins[pin], OUTPUT);
    digitalWrite(R_stepper_pins[pin], LOW);
  }
 
  //set up the servo
  penServo.attach(servoPin);
  
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
#ifdef TURTLE_DEBUG
    Serial.print("AP IP address: ");
    Serial.println(myIP);
#endif
   MDNS.begin(host);
  // Set up the endpoints for HTTP server
  //
  //list directory
  server.on("/list", HTTP_GET, handleFileList);
  //load editor
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
  });
  //create file
  server.on("/edit", HTTP_PUT, handleFileCreate);
  //delete file
  server.on("/edit", HTTP_DELETE, handleFileDelete);
  //first callback is called after the request has ended with all parsed arguments
  //second callback handles file uploads at that location
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);
  server.on("/", HTTP_GET, handle_index);
  server.on("/turtle", HTTP_POST, handle_form);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

