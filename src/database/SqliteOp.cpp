#include "SqliteOp.h"

using std::string;

void DriverDataOp::Open()
{
    int isOpen = sqlite3_open(databaseName.c_str(), &db);
    if (isOpen == SQLITE_OK)
    {
        printf("成功打开 sqlite3 数据库： face_data.db ! \n");
    }
    else
    {
        printf( "无法打开数据库\n");
        ///fprintf(stderr, "无法打开数据库: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
        return;
    }
}

void DriverDataOp::Close()
{
    if (db != NULL)
    {
        sqlite3_close(db);
        db = NULL;
    }
}

void DriverDataOp::CreateDriverTable()
{
    const char *sql = "create table Driver(work_id integer primary key, name varchar(15))";
    char *errMsg = NULL;
    int result = sqlite3_exec(db, sql, NULL, NULL, &errMsg);
    if (result != SQLITE_OK)
    {
        printf("create table Driver failed\n");
        printf("error conde %d \t error message:%s", result, errMsg);
    }
}

void DriverDataOp::InsertDriver(int id, std::string name)
{
    char sql[1024];
    sprintf(sql, "insert into Driver(work_id, name) values (%d, '%s')", id, name.c_str());

    char *errMsg = NULL;
    int result = sqlite3_exec(db, sql, NULL, NULL, &errMsg);
    if (result != SQLITE_OK)
    {
        printf("insert message1:%s \n", errMsg);
    }
}

string DriverDataOp::QueryDriverName(std::string driverID)
{
    char sql[1024];
    sprintf(sql, "select * from Driver where work_id=%s;", driverID.c_str());

    char *errMsg = NULL;

    int nCols;
    int nRows;
    char **azResult;

    int result = sqlite3_get_table(db, sql, &azResult, &nRows, &nCols, &errMsg);
    if (result == SQLITE_OK && nRows > 0)
    {
        string name = azResult[1 * nCols + 1];
        return name;
    }

    return "";
}

void DriverDataOp::QueryAll()
{
    char sql[1024];
    sprintf(sql, "select * from %s;", tableName.c_str());

    char *errMsg = NULL;

    int nCols;
    int nRows;
    char **azResult;

    int result = sqlite3_get_table(db, sql, &azResult, &nRows, &nCols, &errMsg);
    printf("result = %d \t errMsg = %s \n", result, errMsg);
    printf("rows:%d \t cols: %d \n", nRows, nCols);

    printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    for (int i = 0; i < nCols; i++)
    {
        printf("%s\t", azResult[i]);
    }
    printf("\n");

    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            printf("%s\t", azResult[j + (i + 1) * nCols]);  // 专门说明：列头和内容是一起查出来的
        }
        printf("\n");
    }
    printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

    sqlite3_free_table(azResult);
}