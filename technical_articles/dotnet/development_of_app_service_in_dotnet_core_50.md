# .NET core 5.0으로 앱 서비스 개발

## dotnet core 5를 설치

저장소 등록:

```sh
wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
```

설치:

```sh
sudo apt-get update
sudo apt-get install -y apt-transport-https
sudo apt-get update
sudo apt-get install -y dotnet-sdk-5.0
```

참조: https://docs.microsoft.com/en-us/dotnet/core/install/linux-ubuntu

## dotnet entity framework 설치

dotnet 툴 설치:

```sh
dotnet tool install --global dotnet-ef
```

또는 로컬에 설치:

```sh
dotnet tool install dotnet-ef
```

dotnet 툴 업그레이드:

```sh
dotnet tool update --global dotnet-ef
```

또는 로컬에서 업그레이드:

```
dotnet tool update dotnet-ef
```

설치 확인:

```sh
dotnet ef
```

결과:

```
                     _/\__       
               ---==/    \\      
         ___  ___   |.    \|\    
        | __|| __|  |  )   \\\   
        | _| | _|   \_/ |  //|\\ 
        |___||_|       /   \\\/\\

Entity Framework Core .NET Command-line Tools 5.0.7

Usage: dotnet ef [options] [command]

Options:
  --version        Show version information
  -h|--help        Show help information
  -v|--verbose     Show verbose output.
  --no-color       Don't colorize output.
  --prefix-output  Prefix output with level.

Commands:
  database    Commands to manage the database.
  dbcontext   Commands to manage DbContext types.
  migrations  Commands to manage migrations.

Use "dotnet ef [command] --help" for more information about a command.

```

Microsoft.EntityFrameworkCore.Design 패키지 설치:

```sh
dotnet add package Microsoft.EntityFrameworkCore.Design
```

데이터베이스 생성 마이그레이션:

```sh
dotnet ef database update InitialCreate
```

`DbContext`에 대한 코드와 데이터베이스에 대한 엔터티 유형을 생성.

테이블들에 대해 새 Models에 파일 생성 예제:

```sh
dotnet ef dbcontext scaffold "Server=(localdb)\mssqllocaldb;Database=Blogging;Trusted_Connection=True;" Microsoft.EntityFrameworkCore.SqlServer -o Models
```

선택한 테이블에 대해서만 생성 예제:

```sh
dotnet ef dbcontext scaffold "Server=(localdb)\mssqllocaldb;Database=Blogging;Trusted_Connection=True;" Microsoft.EntityFrameworkCore.SqlServer -o Models -t Blog -t Post --context-dir Context -c BlogContext --context-namespace New.Namespace
```

[Secret Manager tool](https://docs.microsoft.com/en-us/aspnet/core/security/app-secrets#secret-manager)에 의해 연결하고 생성:

```sh
dotnet user-secrets set ConnectionStrings:Blogging "Data Source=(localdb)\MSSQLLocalDB;Initial Catalog=Blogging"
dotnet ef dbcontext scaffold Name=ConnectionStrings:Blogging Microsoft.EntityFrameworkCore.SqlServer
```

클래스 외부에서 DbContext를 구성하려는 경우 OnConfiguring 메서드의 스캐 폴딩을 스킵.  ASP.NET Core 앱은 일반적으로 Startup.ConfigureServices에서 구성

```sh
dotnet ef dbcontext scaffold "Server=(localdb)\mssqllocaldb;Database=Blogging;User Id=myUsername;Password=myPassword;" Microsoft.EntityFrameworkCore.SqlServer --no-onconfiguring
```

참조: https://docs.microsoft.com/en-us/ef/core/cli/dotnet

## PostgreSQL 적용

패키지 추가:

```sh
dotnet add package Npgsql.EntityFrameworkCore.PostgreSQL
```

또는 .csproj에 추가:

```xml
<Project>
  <ItemGroup>
    <PackageReference Include="Npgsql.EntityFrameworkCore.PostgreSQL" Version="3.1.3" />
  </ItemGroup>
</Project>
```

`DbContext`에 정의:

```C#
using System.Collections.Generic;
using Microsoft.EntityFrameworkCore;

namespace mynamespace
{
    public class MyDbContext : DbContext
    {
        public DbSet<Blog> Blogs { get; set; }
        public DbSet<Post> Posts { get; set; }

        protected override void OnConfiguring(DbContextOptionsBuilder optionsBuilder)
            => optionsBuilder.UseNpgsql("Host=my_host;Database=my_db;Username=my_user;Password=my_pw");
    }

    public class Blog
    {
        public int BlogId { get; set; }
        public string Url { get; set; }

        public List<Post> Posts { get; set; }
    }

    public class Post
    {
        public int PostId { get; set; }
        public string Title { get; set; }
        public string Content { get; set; }

        public int BlogId { get; set; }
        public Blog Blog { get; set; }
    }
}
```

ASP.NET core의 경우 Startup.cs에 추가 구성:

```C#
public void ConfigureServices(IServiceCollection services)
{
    // Other DI initializations
    services.AddDbContext<MyDbContext>(options =>
            options.UseNpgsql(Configuration.GetConnectionString("MyDbContext")));
}

```

기존 데이터베이스 사용:

```sh
dotnet ef dbcontext scaffold "Host=my_host;Database=my_db;Username=my_user;Password=my_pw" Npgsql.EntityFrameworkCore.PostgreSQL
```

참조: https://www.npgsql.org/efcore/

## 우분투에서 SQL Server로 개발

GPG키 가져오기:

```sh
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
```

Ubuntu 20.04 저장소 등록:

```sh
sudo add-apt-repository "$(wget -qO- https://packages.microsoft.com/config/ubuntu/20.04/mssql-server-2019.list)"
```

SQL 서버 설치:

```sh
sudo apt-get update
sudo apt-get install -y mssql-server
```

SA 패스워드 설정:

```sh
sudo /opt/mssql/bin/mssql-conf setup
```

설치 확인:

```sh
systemctl status mssql-server --no-pager
```

Ubuntu 20.04 CLI 툴 저장소 등록:

```sh
curl https://packages.microsoft.com/config/ubuntu/20.04/prod.list | sudo tee /etc/apt/sources.list.d/msprod.list
```

CLI 툴 설치:

```sh
sudo apt-get update 
sudo apt-get install mssql-tools unixodbc-dev
```

경로 설정:

```sh
echo 'export PATH="$PATH:/opt/mssql-tools/bin"' >> ~/.bash_profile
echo 'export PATH="$PATH:/opt/mssql-tools/bin"' >> ~/.bashrc
source ~/.bashrc
```

로컬에서 연결:

```sh
sqlcmd -S localhost -U SA -P '<YourPassword>'
```

참조: https://docs.microsoft.com/en-us/sql/linux/quickstart-install-connect-ubuntu?view=sql-server-ver15

## Web app Authentication

```
public void ConfigureServices(IServiceCollection services)
{
    services.AddDbContext<ApplicationDbContext>(options =>
        // options.UseSqlite(
        options.UseSqlServer(
            Configuration.GetConnectionString("DefaultConnection")));
    services.AddDatabaseDeveloperPageExceptionFilter();
    services.AddDefaultIdentity<IdentityUser>(options => options.SignIn.RequireConfirmedAccount = true)
        .AddEntityFrameworkStores<ApplicationDbContext>();
    services.AddRazorPages();

    services.Configure<IdentityOptions>(options =>
    {
        // Password settings.
        options.Password.RequireDigit = true;
        options.Password.RequireLowercase = true;
        options.Password.RequireNonAlphanumeric = true;
        options.Password.RequireUppercase = true;
        options.Password.RequiredLength = 6;
        options.Password.RequiredUniqueChars = 1;

        // Lockout settings.
        options.Lockout.DefaultLockoutTimeSpan = TimeSpan.FromMinutes(5);
        options.Lockout.MaxFailedAccessAttempts = 5;
        options.Lockout.AllowedForNewUsers = true;

        // User settings.
        options.User.AllowedUserNameCharacters =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-._@+";
        options.User.RequireUniqueEmail = false;
    });

    services.ConfigureApplicationCookie(options =>
    {
        // Cookie settings
        options.Cookie.HttpOnly = true;
        options.ExpireTimeSpan = TimeSpan.FromMinutes(5);

        options.LoginPath = "/Identity/Account/Login";
        options.AccessDeniedPath = "/Identity/Account/AccessDenied";
        options.SlidingExpiration = true;
    });
}
```



```C#
public void Configure(IApplicationBuilder app, IWebHostEnvironment env)
{
    if (env.IsDevelopment())
    {
        app.UseDeveloperExceptionPage();
        app.UseMigrationsEndPoint();
    }
    else
    {
        app.UseExceptionHandler("/Error");
        app.UseHsts();
    }

    app.UseHttpsRedirection();
    app.UseStaticFiles();

    app.UseRouting();

    app.UseAuthentication();
    app.UseAuthorization();

    app.UseEndpoints(endpoints =>
    {
        endpoints.MapRazorPages();
    });
}
```

코드 생성:

```sh
dotnet add package Microsoft.VisualStudio.Web.CodeGeneration.Design
dotnet aspnet-codegenerator identity -dc WebApp1.Data.ApplicationDbContext --files "Account.Register;Account.Login;Account.Logout;Account.RegisterConfirmation"
```





참조: https://docs.microsoft.com/en-us/aspnet/core/security/authentication/identity?view=aspnetcore-5.0&tabs=netcore-cli

## Web app Authorization







참조: https://docs.microsoft.com/en-us/aspnet/core/security/authorization/secure-data?view=aspnetcore-5.0



## 웹앱 생성

https://docs.microsoft.com/en-us/aspnet/core/security/authorization/secure-data?view=aspnetcore-5.0#create-the-starter-app

## 참조

- https://docs.microsoft.com/en-us/dotnet/core/install/linux-ubuntu