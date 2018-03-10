import MySQLdb


def connect():
    con = MySQLdb.connect(host='localhost', user='root', passwd='DUKS1992', db='ROBOT')
    try:

        cur = con.cursor()
        # query = "INSERT INTO SLAM(Time_stamp_big ,Scan_data ,Partical_pos ,Errors , Cylinders ,Error_elipses ,Time_Stamp_end ,Opration_time) values(\"gjfig\",\"fhgh\",\"gfdhg\",\"dhdh\",\"hhg\",\"fghfh\",\"dgdg\",2)"
        # cur.execute(query)
        con.commit()
        # data = cur.fetchall()e
    except MySQLdb.Error, e:
        print(e)
        if con:
            con.rollback()
    return con

# ........................................................................
def SQL_CMD(con, query):
    cur = con.cursor()
    cur.execute(query)
    con.commit()


# .........................................................................
def SQL_CMD_getData(con ,query):
    cur = con.cursor()
    cur.execute(query)
    result = cur.fetchall()
    return result



con = connect()
print con

#quary = "SELECT * FROM Data_ref WHERE is_New = '1' LIMIT 1"
#result = SQL_CMD_getData(con, quary)
#num  =  result[0][0]
#print num
#quary_get ="SELECT * FROM SLAM WHERE  No = '"+str(result[0][0])+"'"
#result2 = SQL_CMD_getData(con,quary_get)
#print [int(i) for i in result2[0][1].strip('[]').split(',')]# scan data
#rint [float(i.strip('.')) for i in result2[0][2].strip('[]').split(',')]#postion
#print [[float(r) for r in i.split(',')] for i in result2[0][3].strip('()').strip('[()]').split('), (')]# parical pos
#print [float(r) for r in result2[0][4].strip('()').split(',')]# errors
#print [[float(r) for r  in str(i).split(', ')]  for i in result2[0][5].strip('[[]]').split('], [')]#cylinder pos
#print [ [float(r) for r  in i.strip('()').split(',')] for i in result2[0][6].strip('[]').split('), (')] # error elipse

#print result2[0][6]

#quary2 = "UPDATE Data_ref SET is_New = '0' WHERE No = '"+str(num)+"'"
#SQL_CMD(con,quary2)

quary3 = "UPDATE Data_ref SET is_New = '1'"
SQL_CMD(con,quary3)