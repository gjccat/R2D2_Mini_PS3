#ifndef ARDUINO_R2HTML_H
#define ARDUINO_R2HTML_H
#include <Arduino.h>

class r2HTML
{
private:
    String header(String Title); 
    String menu();
    String menuStyle();
    String style();

    String footer = R"rawliteral(</div></body>
</html>
)rawliteral";




    String getInputScript(String functionName, String handelerPath);
    String getInputScriptMAC(String functionName, String handelerPath);
    

public:
   String getPWMOutputSetupHTML();
    String getIndexHTML();
    String getpsControllerHTML();
    r2HTML();
};

#if !defined(NO_GLOBAL_INSTANCES)
extern r2HTML R2D2HTML;
#endif
#endif