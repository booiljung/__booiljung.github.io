# [Oat++](https://oatpp.io/)

## HTTP API

### .vscode/c_cpp_properties.json

`inlcudePath` 내용:

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/local/include/oatpp-1.3.0",
                "/usr/local/include/oatpp-1.3.0/oatpp",
                "/usr/local/include/oatpp-1.3.0/oatpp/oatpp/codegen",
                "/usr/local/include/oatpp-1.3.0/oatpp-curl",
                "/usr/local/include/oatpp-1.3.0/oatpp"
            ],
            ...
        }
    ],
    "version": 4
}
```

### dto/MyDTOs.hpp

```c++
#include "oatpp/core/Types.hpp"
#include "oatpp/core/macro/codegen.hpp"

#include OATPP_CODEGEN_BEGIN(DTO)

class HelloDto : public oatpp::DTO {
    DTO_INIT(HelloDto, DTO)
    
  	DTO_FIELD(String, userAgent, "user-agent");
  	DTO_FIELD(String, message);
  	DTO_FIELD(String, server);          
}

#include OATPP_CODEGEN_END(DTO)
```

### AppComponent.hpp

```c++
#include "oatpp/web/server/AsyncHttpConnectionHandler.hpp"
#include "oatpp/web/server/HttpRouter.hpp"
#include "oatpp/network/tcp/server/ConnectionProvider.hpp"
#include "oatpp/parser/json/mapping/ObjectMapper.hpp"
#include "oatpp/core/macro/component.hpp"
```

애플리케이션 컴포넌트를 생성 및 보관하고 `oatpp::base::Environment`에 컴포넌트를 등록하는 클래스다. 컴포넌트 초기화 순서는 위에서 아래로다:

```c++
class AppComponent
{
public:
```

`async::Executor` 생성:

```c++
  OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::async::Executor>, executor)
  ([]{
    return std::make_shared<oatpp::async::Executor>(
      9 /* Data-Processing threads */,
      2 /* I/O threads */,
      1 /* Timer threads */
    );
  }());
```

포트를 listen 하게 될 `network::ServerConnectionProvider`  생성한다.  논블로킹 연결은 `AsyncIO`를 위해 `AsyncHttpConnectionHandler`를 사용해야 한다:

```c++
  OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::network::ServerConnectionProvider>, serverConnectionProvider)
  ([]{
    return oatpp::network::tcp::server::ConnectionProvider::createShared({"0.0.0.0", 8000, oatpp::network::Address::IP_4});
  }());
```

HTTP 라우터 컴포넌트를 생성한다:

```c++
  OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::web::server::HttpRouter>, httpRouter)
  ([]{
    return oatpp::web::server::HttpRouter::createShared();
  }());
```

`Router` 컴포넌트를 사용하여 요청을 라우팅하는 `ConnectionHandler` 컴포넌트 생성:

```c++
  OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::network::ConnectionHandler>, serverConnectionHandler)
  ([]{
    OATPP_COMPONENT(std::shared_ptr<oatpp::web::server::HttpRouter>, router); // get Router component
    OATPP_COMPONENT(std::shared_ptr<oatpp::async::Executor>, executor); // get Async executor component
    return oatpp::web::server::AsyncHttpConnectionHandler::createShared(router, executor);
  }());
```

`Contoller`의 API에서 DTO를 직렬화/역직렬화하기 위해 `ObjectMapper` 컴포넌트를 생성:

```c++
  OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::data::mapping::ObjectMapper>, apiObjectMapper)
  ([]{
    auto serializerConfig = oatpp::parser::json::mapping::Serializer::Config::createShared();
    auto deserializerConfig = oatpp::parser::json::mapping::Deserializer::Config::createShared();
    deserializerConfig->allowUnknownFields = false;
    auto objectMapper = oatpp::parser::json::mapping::ObjectMapper::createShared(serializerConfig, deserializerConfig);
    return objectMapper;
  }());
};
```

### MyController

#### MyController.hpp

```c++

#ifndef MyController_hpp
#define MyController_hpp

#include "../dto/MyDTOs.hpp"

#include "oatpp/web/server/api/ApiController.hpp"
#include "oatpp/core/macro/codegen.hpp"
#include "oatpp/core/macro/component.hpp"

#include OATPP_CODEGEN_BEGIN(ApiController) //<-- Begin codegen

/**
 *  EXAMPLE ApiController
 *  Basic examples of howto create ENDPOINTs
 *  More details on oatpp.io
 */
class MyController : public oatpp::web::server::api::ApiController
{
protected:
  MyController(const std::shared_ptr<ObjectMapper> &objectMapper)
      : oatpp::web::server::api::ApiController(objectMapper)
  {
  }

public:
  /**
   *  Inject @objectMapper component here as default parameter
   *  Do not return bare Controllable* object! use shared_ptr!
   */
  static std::shared_ptr<MyController> createShared(OATPP_COMPONENT(std::shared_ptr<ObjectMapper>,
                                                                    objectMapper))
  {
    return std::shared_ptr<MyController>(new MyController(objectMapper));
  }

  /**
   *  Hello World endpoint Coroutine mapped to the "/" (root)
   */
  ENDPOINT_ASYNC("GET", "/", Root){

      ENDPOINT_ASYNC_INIT(Root)

      /**
       *  Coroutine entrypoint act()
       *  returns Action (what to do next)
       */
      Action act() override{
          auto dto = HelloDto::createShared();
  dto->message = "Hello Async!";
  dto->server = Header::Value::SERVER;
  dto->userAgent = request->getHeader(Header::USER_AGENT);
  return _return(controller->createDtoResponse(Status::CODE_200, dto));
}
}
;

/**
 *  Echo body endpoint Coroutine. Mapped to "/body/string".
 *  Returns body received in the request
 */
ENDPOINT_ASYNC("GET", "/body/string", EchoStringBody){

    ENDPOINT_ASYNC_INIT(EchoStringBody)

        Action act() override{
            /* return Action to start child coroutine to read body */
            return request->readBodyToStringAsync().callbackTo(&EchoStringBody::returnResponse);
}

Action returnResponse(const oatpp::String &body)
{
  /* return Action to return created OutgoingResponse */
  return _return(controller->createResponse(Status::CODE_200, body));
}
}
;

/**
 *  Echo body endpoint Coroutine. Mapped to "/body/dto".
 *  Deserialize DTO reveived, and return same DTO
 *  Returns body as MessageDto received in the request
 */
ENDPOINT_ASYNC("GET", "/body/dto", EchoDtoBody){

    ENDPOINT_ASYNC_INIT(EchoDtoBody)

        Action act() override{
            return request->readBodyToDtoAsync<oatpp::Object<MessageDto>>(controller->getDefaultObjectMapper()).callbackTo(&EchoDtoBody::returnResponse);
}

Action returnResponse(const oatpp::Object<MessageDto> &body)
{
  return _return(controller->createDtoResponse(Status::CODE_200, body));
}
}
;
}
;

#include OATPP_CODEGEN_BEGIN(ApiController) //<-- End codegen

#endif /* MyController_hpp */

```

#### MyController.cpp

```c++
#include "MyController.hpp"
// TODO some code here
```

### App.cpp

```c++
#include <iostream>
#include "oatpp/network/Server.hpp"
#include "./controller/MyController.hpp"
#include "./AppComponent.hpp"
```

`run` 메소드, set `Environment` components, add `ApiController`'s endpoints to router, run server:

```c++
void run() {
```

Create scope Environment components:

```c++
  AppComponent components;
```

Create `ApiControllers` and add endpoints to router:

```c++
  auto router = components.httpRouter.getObject();
  router->addController(MyController::createShared());
```

Create server:

```c++
  oatpp::network::Server server(components.serverConnectionProvider.getObject(),
                                components.serverConnectionHandler.getObject()); 
  OATPP_LOGD("Server", "Running on port %s...", components.serverConnectionProvider.getObject()->getProperty("port").toString()->c_str());
```

서버 실행:

```c++
  server.run();  
}
```

main 메소드:

```c++
int main(int argc, const char * argv[]) {
```

oat++ 환경 초기화:

```c++
  oatpp::base::Environment::init();
```

실행:

```c++
  run();
```

앱 실행 중에 생성된 오브젝트 수와 누출 가능성이 있는 남은 오브젝트 수 인쇄. 성능 향상을 위해 `-D OATPP_DISABLE_ENV_OBJECT_COUNTS` 플래그를 사용하여 릴리스 빌드에서 오브젝트 카운팅을 비활성화 할 수 있다:

```c++
  std::cout << "\nEnvironment:\n";
  std::cout << "objectsCount = " << oatpp::base::Environment::getObjectsCount() << "\n";
  std::cout << "objectsCreated = " << oatpp::base::Environment::getObjectsCreated() << "\n\n";
```

oat++ 환경 해제:

```c++
  oatpp::base::Environment::destroy();  
  return 0;
}
```

## Async WebSocket Server

### AppComponent.hpp

```c++
#include "oatpp/web/server/AsyncHttpConnectionHandler.hpp"
#include "oatpp/web/server/HttpRouter.hpp"
#include "oatpp/network/tcp/server/ConnectionProvider.hpp"
#include "oatpp/parser/json/mapping/ObjectMapper.hpp"
#include "oatpp/core/macro/component.hpp"

#include "./1websocket/WSListener.hpp"
```

AppComponent는 애플리케이션 컴포넌트를 생성 및 보유하고 oatpp::base::Environment에 컴포넌트를 등록하는 클래스다. 컴포넌트 초기화 순서는 위에서 아래로다.

```c++
class AppComponent
{
public:
```

Async Executor를 생성한다:

```c++
OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::async::Executor>, executor)
([]{
  return std::make_shared<oatpp::async::Executor>(
    4 /* Data-Processing threads */,
    1 /* I/O threads */,
    1 /* Timer threads */
  );
}());
```

포트를 리슨할 ConnectionProvider를 생성한다:

```c++
OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::network::ServerConnectionProvider>, serverConnectionProvider)
([]{
  return oatpp::network::tcp::server::ConnectionProvider::createShared(
    {"0.0.0.0", 8000, oatpp::network::Address::IP_4}
  );
}());
```

라우터 콤포넌트를 생성한다:

```C++
OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::web::server::HttpRouter>, httpRouter)
([]{
  return oatpp::web::server::HttpRouter::createShared();
}());
```

라우터 컴포넌트를 사용하여 요청을 라우팅하는 ConnectionHandler 컴포넌트를 생성:

```c++
OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::network::ConnectionHandler>, serverConnectionHandler)
("http",[]{
  // get Router component
  OATPP_COMPONENT(std::shared_ptr<oatpp::web::server::HttpRouter>, router); 
  // get Async executor component
  OATPP_COMPONENT(std::shared_ptr<oatpp::async::Executor>, executor);
  return oatpp::web::server::AsyncHttpConnectionHandler::createShared(router, executor);
}());
```

컨트롤러의 API에서 DTO를 직렬화/역직렬화하기 위해 ObjectMapper 컴포넌트를 생성:

```c++
OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::data::mapping::ObjectMapper>, apiObjectMapper)
([]{
  return oatpp::parser::json::mapping::ObjectMapper::createShared();
}());
```

웹소켓 연결 핸들러 생성:

```C++
OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::network::ConnectionHandler>, websocketConnectionHandler)
("websocket",[]{
  OATPP_COMPONENT(std::shared_ptr<oatpp::async::Executor>, executor);
  auto connectionHandler = oatpp::websocket::AsyncConnectionHandler::createShared(executor);
  connectionHandler->setSocketInstanceListener(std::make_shared<WSInstanceListener>());
  return connectionHandler;
}());
```

AppComponent 클래스 끝:

```c++
};
```

### App.cpp

```c++
#include <iostream>
#include "oatpp/network/Server.hpp"
#include "./controller/MyController.hpp"
#include "./AppComponent.hpp"
```

run 함수:

```c++
void run() {
```

run 메소드 스코프 내에서 컴포넌트들을 등록:

```c++
  AppComponent components;
```

라우터 컴포넌트 얻기:

```
  OATPP_COMPONENT(std::shared_ptr<oatpp::web::server::HttpRouter>, router);
```

MyController를 생성하고 모든 엔드포인트를 라우터에 추가:

```c++
  router->addController(std::make_shared<MyController>());
```

ConnectionHandler 컴포넌트 가져오기:

```c++
  OATPP_COMPONENT(std::shared_ptr<oatpp::network::ConnectionHandler>, connectionHandler, "http");
```

ConnectionProvider 컴포넌트 가져오기:

```
  OATPP_COMPONENT(std::shared_ptr<oatpp::network::ServerConnectionProvider>, connectionProvider);
```













  /* Create server which takes provided TCP connections and passes them to HTTP connection handler */

  oatpp::network::Server server(connectionProvider, connectionHandler);

  /* Priny info about server port */

  OATPP_LOGI("MyApp", "Server running on port %s", connectionProvider->getProperty("port").getData());

  /* Run server */

  server.run();

}

int main(int argc, const char * argv[]) {

  oatpp::base::Environment::init();

  run();

  oatpp::base::Environment::destroy();

  return 0;

}





