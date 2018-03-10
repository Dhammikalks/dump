
import Pyro4
import sys,time


#greeting_maker = Pyro4.Proxy("PYRONAME:example.avi")    # use name server object lookup uri shortcut
#count = 0;
#while True:
#    print(greeting_maker.data(count))
#    count = count+1
#    time.sleep(0.5)
# saved as greeting-client.py
import Pyro4

name = "dhammika" #input("What is your name? ").strip()

greeting_maker = Pyro4.Proxy("PYRONAME:example.greeting")    # use name server object lookup uri shortcut
print(greeting_maker)
count = 1
while True:
    if greeting_maker.is_avilable():

        print greeting_maker.is_avilable()
        data = greeting_maker.get_Data();
        greeting_maker.false_avilable()

        print(data[0])
        print(data[1])
        print(data[2])
        print(data[3])
        print(data[4])
        print(data[5])
        print("----------------------------------")


    time.sleep(0.5)

