#include "crow.h"

int main(){
    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([](){
        return "Hellow world";
    });
    
    app.port(18080).multithreaded().run();
}