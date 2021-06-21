# SQL Server with Docker-Compose on Ubuntu Focal

## docker-compose.yml

```docker-compose
sqldata:
    image: mcr.microsoft.com/mssql/server:2019-CU11-ubuntu-20.04
    environment:
        - SA_PASSWORD=<password>
        - ACCEPT_EULA=Y
        - MSSQL_PID=Developer # This will run the container using the Developer Edition (this is the default if no MSSQL_PID environment variable is supplied)
    ports:
        - "11433:1433"
    volumes:
        - <host directory>/data:/var/opt/mssql/data
        - <host directory>/log:/var/opt/mssql/log
        - <host directory>/secrets:/var/opt/mssql/secrets
```

