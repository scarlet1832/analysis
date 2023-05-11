# uncompyle6 version 3.9.0
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.9.13 (main, Aug 25 2022, 23:26:10) 
# [GCC 11.2.0]
# Embedded file name: saegobetter/DATA_ANALYSIS_Demo/src/ExcelOperation.py
import xlsxwriter as xw

def WritetToExcel(data, fileName):
    workbook = xw.Workbook(fileName + '.xlsx')
    worksheet1 = workbook.add_worksheet('sheet1')
    worksheet1.activate()
    i = 1
    for keys, values in data.items():
        if values != 'NoResult':
            print(keys)
            for key, value in values.items():
                print(key, value)
                row = 'A' + str(i)
                worksheet1.write_row(row, [str(key), str(value)])
                i += 1

    workbook.close()

# def WritetToExcel(data, fileName):
#     workbook = xw.Workbook(fileName + '.xlsx')
#     worksheet1 = workbook.add_worksheet('sheet1')
#     worksheet1.activate()
#     i = 1
#     for row in data:
#         if len(row) > 1:
#             # print(len(row))
#             for k in range(0, len(row) - 1, 2):
#                 worksheet1.write(i, 0, str(row[k]))  # write to column A
#                 worksheet1.write(i, 1, str(row[k + 1]))  # write to column B
#                 i += 1

    workbook.close()


if __name__ == '__main__':
    testData = [
     [
      'NoRsult'],
     [
      'intensity', [5, 3, 4, 6, 87, 2342, 345, 56543]],
     [
      'asd', 123, 'asd', 123]]
    fileName = '测试result.xlsx'
    WritetToExcel(testData, fileName)
    print('end')