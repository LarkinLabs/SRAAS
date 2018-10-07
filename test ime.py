import datetime
while True:
    now = datetime.datetime.now()
    Nowtime = str("%d%d%d"%(now.minute,now.second,now.microsecond))
