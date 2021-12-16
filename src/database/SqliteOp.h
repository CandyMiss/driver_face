#ifndef DRIVER_MONITOR_SQLITEOP_H
#define DRIVER_MONITOR_SQLITEOP_H

#include <string>
#include <sqlite3.h>

class DriverDataOp
{
private:
    const std::string databaseName = "face_data.db";
    const std::string tableName = "Driver";
    sqlite3 *db = NULL;
public:
    DriverDataOp()
    {
    }

    void Open();

    void Close();

    void CreateDriverTable();

    void InsertDriver(int id, std::string name);

    std::string QueryDriverName(std::string driverID);

    void QueryAll(); // 测试函数
};

#endif //DRIVER_MONITOR_SQLITEOP_H
