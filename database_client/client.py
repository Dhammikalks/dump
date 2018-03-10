import MySQLdb

def connect_SLAM():

    con = MySQLdb.connect(host='localhost', user='root', passwd='DUKS1992', db='ROBOT')
    try:

        cur = con.cursor()
        query ="INSERT INTO SLAM(Time_stamp_big ,Scan_data ,Partical_pos ,Errors , Cylinders ,Error_elipses ,Time_Stamp_end ,Opration_time) values(\"gjfig\",\"fhgh\",\"gfdhg\",\"dhdh\",\"hhg\",\"fghfh\",\"dgdg\",2)"
        cur.execute(query)
        con.commit()
        #data = cur.fetchall()e
    except MySQLdb.Error, e:
        print(e)
        if con:
            con.rollback()



connect_SLAM()