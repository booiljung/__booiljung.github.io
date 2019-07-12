# 2. Reading from a Database

We will continue to build on the last chapter's project, `heroes`, by storing our heroes in a database. This will let us to edit our heroes and keep the changes when we restart the application.

## Object-Relational Mapping

A relational database management system (like PostgreSQL or MySQL)  stores its data in the form of tables. A table represents some sort of  entity - like a person or a bank account. Each table has columns that  describe the attributes of that entity - like a name or a balance. Every  row in a table is an instance of that entity - like a single person  named Bob or a bank account.

In an object-oriented framework like Aqueduct, we have  representations for tables, columns and rows. A class represents a  table, its instances are rows, and instance properties are column  values. An ORM translates rows in a database to and from objects in an  application.

| Aqueduct     | Database   | Example #1         | Example #2           |
| ------------ | ---------- | ------------------ | -------------------- |
| **Class**    | **Table**  | Person             | Bank Account         |
| **Instance** | **Row**    | A person named Bob | Sally's Bank Account |
| **Property** | **Column** | Person's Name      | Bank Account Balance |

In Aqueduct, each database table-class pairing is called an *entity*. Collectively, an application's entities are called its *data model*.

## Building a Data Model

In our `heroes` application, we will have one type of entity - a "hero". To create a new entity, we subclass `ManagedObject<T>`. Create a new directory `lib/model/` and then add a new file to this directory named `hero.dart`. Add the following code:



```
import 'package:heroes/heroes.dart';

class Hero extends ManagedObject<_Hero> implements _Hero {}

class _Hero {
  @primaryKey
  int id;

  @Column(unique: true)
  String name;
}
```

This declares a `Hero` entity. Entities are always made up of two classes.

The `_Hero` class is a direct mapping of a database table. This table's name will have the same name as the class: `_Hero`. Every property declared in this class will have a corresponding column in this table. Therefore, the `_Hero` table will have two columns - `id` and `name`. The `id` column is this table's primary key (a unique identifier for each hero). The name of each hero must be unique.

The other class, `Hero`, is what we work with in our code - when we fetch heroes from a database, they will be instances of `Hero`.

The `Hero` class is called the *instance type* of the entity, because that's what we have instances of. `_Hero` is the *table definition* of the entity. You won't use the table definition for anything other than describing the database table.

An instance type must *implement* its table definition; this gives our `Hero` all of the properties of `_Hero`. An instance type must *extend* `ManagedObject<T>`, where `T` is also the table definition. `ManagedObject<T>` has behavior for automatically transferring objects to the database and back (among other things).

Transient Properties

Properties declared in the instance type aren't stored in the  database. This is different than properties in the table definition. For  example, a database table might have a `firstName` and `lastName`, but it's useful in some places to have a `fullName` property. Declaring the `fullName`  property in the instance type means we have easy access to the full  name, but we still store the first and last name individually.

## Defining a Context

Our application needs to know two things to execute database queries:

1. What is the data model (our collection of entities)?
2. What database are we connecting to?

Both of these things are set up when an application is first started. In `channel.dart`, add a new property `context` and update `prepare()`:



```
class HeroesChannel extends ApplicationChannel {
  ManagedContext context;

  @override
  Future prepare() async {
    logger.onRecord.listen((rec) => print("$rec ${rec.error ?? ""} ${rec.stackTrace ?? ""}"));

    final dataModel = ManagedDataModel.fromCurrentMirrorSystem();
    final persistentStore = PostgreSQLPersistentStore.fromConnectionInfo(
      "heroes_user", "password", "localhost", 5432, "heroes");

    context = ManagedContext(dataModel, persistentStore);
  }

  @override
  Controller get entryPoint {
    ...
```

`ManagedDataModel.fromCurrentMirrorSystem()` will find all of our `ManagedObject<T>` subclasses and 'compile' them into a data model. A `PostgreSQLPersistentStore`  takes database connection information that it will use to connect and  send queries to a database. Together, these objects are packaged in a `ManagedContext`.

Configuring a Database Connection

This tutorial hardcodes the information needed to connect to a  database. In a future chapter, we will move these values to a  configuration file so that we can change them during tests and various  deployment environments.

The context will coordinate with these two objects to execute queries  and translate objects to and from the database. Controllers that make  database queries need a reference to the context. So, we'll want `HeroesController` to have access to the context.

In `heroes_controller.dart`, add a property and create a new constructor:



```
class HeroesController extends ResourceController {
  HeroesController(this.context);

  final ManagedContext context;  
  ...
```

Now that `HeroesController` requires a context in its constructor, we need to pass it the context we created in `prepare()`. Update `entryPoint` in `channel.dart`.



```
@override
Controller get entryPoint {
  final router = Router();

  router
    .route("/heroes/[:id]")
    .link(() => HeroesController(context));

  router
    .route("/example")
    .linkFunction((request) async {
      return new Response.ok({"key": "value"});
  });

  return router;
}
```

Now that we've 'injected' this context into our `HeroesController` constructor, each `HeroesController` can execute database queries.

Service Objects and Dependency Injection

Our context is an example of a *service object*. A service  encapsulates logic and state into a single object that can be reused in  multiple controllers. A typical service object accesses another server,  like a database or another REST API. Some service objects may simply  provide a simplified interface to a complex process, like applying  transforms to an image. Services are passed in a controller's  constructor; this is called *dependency injection*. Unlike many frameworks,  Aqueduct does not require a complex dependency injection framework; this  is because you write the code to create instances of your controllers  and can pass whatever you like in their constructor.

## Executing Queries

Our operation methods in `HeroesController` currently  return heroes from an in-memory list. To fetch data from a database  instead of this list, we create and execute instances of `Query<T>` in our `ManagedContext`.

Let's start by replacing `getAllHeroes` in `heroes_controller.dart`. Make sure to import your `model/hero.dart` file at the top:



```
import 'package:heroes/heroes.dart';
import 'package:heroes/model/hero.dart';

class HeroesController extends ResourceController {
  HeroesController(this.context);

  final ManagedContext context;

  @Operation.get()
  Future<Response> getAllHeroes() async {
    final heroQuery = Query<Hero>(context);
    final heroes = await heroQuery.fetch();

    return Response.ok(heroes);
  }

...
```

Here, we create an instance of `Query<Hero>` and then execute its `fetch()` method. The type argument to `Query<T>`  is an instance type; it lets the query know which table to fetch rows  from and the type of objects that are returned by the query. The context  argument tells it which database to fetch it from. The `fetch()` execution method returns a `List<Hero>`. We write that list to the body of the response.

Now, let's update `getHeroByID` to fetch a single hero from the database.



```
@Operation.get('id')
Future<Response> getHeroByID(@Bind.path('id') int id) async {
  final heroQuery = Query<Hero>(context)
    ..where((h) => h.id).equalTo(id);    

  final hero = await heroQuery.fetchOne();

  if (hero == null) {
    return Response.notFound();
  }
  return Response.ok(hero);
}
```

This query does two interesting things. First, it uses the `where` method to filter heroes that have the same `id` as the path variable. For example, `/heroes/1` will fetch a hero with an `id` of `1`. This works because `Query.where` adds a SQL WHERE clause to the query. We'd get the following SQL:



```
SELECT id, name FROM _question WHERE id = 1;
```

The `where` method uses the *property selector*  syntax. This syntax is a closure that takes an argument of the type  being queried, and must return a property of that object. This creates  an expression object that targets the selected property. By invoking  methods like `equalTo` on this expression object, a boolean expression is added to the query.

Property Selectors

Many query configuration methods use the property selector syntax.  Setting up a keyboard shortcut (called a Live Template in IntelliJ) to  enter the syntax is beneficial. A downloadable settings configuration  for IntelliJ exists [here](https://aqueduct.io/docs/intellij/) that includes this shortcut.

The `fetchOne()` execution method will fetch a single  object that fulfills all of the expressions applied to the query. If no  database row meets the criteria, `null` is returned. Our controller returns a 404 Not Found response in that scenario.

We have now written code that fetches heroes from a database instead of from in memory, but we don't have a database - yet.

fetchObjectWithID, fetchOne() and Unique Properties

You can also fetch an object by its primary key with the method `ManagedContext.fetchObjectWithID`. When fetching with `fetchOne`, make sure the search criteria is guaranteed to be unique.

## Setting Up a Database

For development, you'll need to install a PostgreSQL server on your local machine. If you are on macOS, use [Postgres.app](http://postgresapp.com). This native macOS application manages starting and stopping PostgreSQL servers on your machine. For other platforms, see [this page](https://www.postgresql.org/download/).

9.6 or Greater

The minimum version of PostgreSQL needed to work with Aqueduct is 9.6.

If you installed Postgres.app, open the application and select the `+`  button on the bottom left corner of the screen to create a new database  server. Choose a version (at least 9.6, but the most recent version is  best), name the server whatever you like, and leave the rest of the  options unchanged before clicking `Create Server`. Once the server has been created, click `Start`.

A list of databases available on this server will be shown as named, database icons. Double-click on any of them to open the `psql` command-line tool.

psql

For other platforms, `psql` should be available in your `$PATH`. You can also add `Postgres.app`'s `psql` to your path with the directions [here](https://postgresapp.com/documentation/cli-tools.html).

In `psql`, create a new database and a user to manage it.



```
CREATE DATABASE heroes;
CREATE USER heroes_user WITH createdb;
ALTER USER heroes_user WITH password 'password';
GRANT all ON database heroes TO heroes_user;
```

Next, we need to create the table where heroes are stored in this  database. From your project directory, run the following command:



```
aqueduct db generate
```

This command will create a new *migration file*. A migration  file is a Dart script that runs a series of SQL commands to alter a  database's schema. It is created in a new directory in your project  named `migrations/`. Open `migrations/00000001_initial.migration.dart`, it should look like this:



```
import 'package:aqueduct/aqueduct.dart';
import 'dart:async';

class Migration1 extends Migration {
  @override
  Future upgrade() async {
    database.createTable(SchemaTable(
      "_Hero", [
        SchemaColumn("id", ManagedPropertyType.bigInteger,
            isPrimaryKey: true, autoincrement: true, isIndexed: false, isNullable: false, isUnique: false),
        SchemaColumn("name", ManagedPropertyType.string,
            isPrimaryKey: false, autoincrement: false, isIndexed: false, isNullable: false, isUnique: true),
      ],
    ));
  }

  @override
  Future downgrade() async {}

  @override
  Future seed() async {}
}
```

In a moment, we'll execute this migration file. That will create a new table named `_Hero` with columns for `id` and `name`. Before we run it, we should seed the database with some initial heroes. In the `seed()` method, add the following:



```
@override
Future seed() async {
  final heroNames = ["Mr. Nice", "Narco", "Bombasto", "Celeritas", "Magneta"];

  for (final heroName in heroNames) {    
    await database.store.execute("INSERT INTO _Hero (name) VALUES (@name)", substitutionValues: {
      "name": heroName
    });
  }
}
```

Apply this migration file to our locally running `heroes` database with the following command in the project directory:



```
aqueduct db upgrade --connect postgres://heroes_user:password@localhost:5432/heroes
```

Re-run your application with `aqueduct serve`. Then, reload <http://aqueduct-tutorial.stablekernel.io>. Your dashboard of heroes and detail page for each will still show up - but this time, they are sourced from a database.

ManagedObjects and Migration Scripts

In our migration's `seed()` method, we executed SQL queries instead of using the Aqueduct ORM. *It is very important that you do not use* `Query<T>`, `ManagedObject<T>`  or other elements of the Aqueduct ORM in migration files. Migration  files represent an ordered series of historical steps that describe your  database schema. If you replay those steps (which is what executing a  migration file does), you will end up with the same database schema  every time. However, a `ManagedObject<T>` subclass  changes over time - the definition of a managed object is not  historical, it only represents the current point in time. Since a `ManagedObject<T>` subclass can change, using one in our migration file would mean that our migration file could change.

## Query Parameters and HTTP Headers

In the browser application, the dashboard has a text field for  searching heroes. When you enter text into it, it will send the search  term to the server by appending a query parameter to `GET /heroes`. For example, if you entered the text `abc`, it'd make this request:



```
GET /heroes?name=abc
```

![Aqueduct Tutorial Run 4](https://aqueduct.io/docs/img/run4.png)

Aqueduct 응용 프로그램은 이 값을 사용하여 검색 문자열이 들어있는 heroes의 목록을 반환 할 수 있습니다. `heroes_controller.dart`에서`name` 쿼리 매개 변수를 바인딩하기 위해 `getAllHeroes()`를 수정하십시오 :

```dart
@Operation.get()
Future<Response> getAllHeroes({@Bind.query('name') String name}) async {
  final heroQuery = Query<Hero>(context);
  if (name != null) {
    heroQuery.where((h) => h.name).contains(name, caseSensitive: false);
  }
  final heroes = await heroQuery.fetch();

  return Response.ok(heroes);
}
```

Aqueduct 응용 프로그램을 다시 실행하고 클라이언트 응용 프로그램의 검색 표시 줄을 사용할 수 있습니다.

`@Bind.query('name')` 주석은 요청 URL에 포함 된  `'name'` 쿼리 매개 변수의 값을 바인딩 합니다. 그렇지 않으면`name`은 null이 됩니다.

`name`은 *선택적 매개 변수* (중괄호로 묶여 있음)입니다. 오퍼레이션 메소드의 선택적 매개 변수는 HTTP 요청에서도 선택적입니다. 이 바인딩에서 중괄호를 제거하면 `'name'` 쿼리 매개 변수가 필요하게 되고 `?name = x'`이 없는 `GET /heroes` 요청은 400 Bad Request로 실패합니다.

ResourceController Binding

바운드 값을 `int`와 `DateTime` 같은 타입으로 자동적으로 파싱하는 것과 같이 우리가 보여준 것보다 더 많은 바인딩이 있습니다. 자세한 내용은 [ResourceController](https://aqueduct.io/docs/http/resource_controller/)를 참조하십시오.

오퍼레이션 메서드에서 쿼리 및 헤더 매개 변수를 바인딩하면 코드를 의도적으로 만들고 공용 코드 파싱 코드가 발생하지 않도록 하는 좋은 방법입니다. Aqueduct는 바인딩을 사용할 때 더 나은 문서를 생성 할 수 있습니다.