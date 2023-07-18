
#$language = "Python"
#$interface = "1.0"
 
def Main():
    tab = crt.GetScriptTab()
 
    if tab.Session.Connected != True:
        crt.Dialog.MessageBox(
            "Error.\n" +
            "This script was designed to be launched after a valid "+
            "connection is established.\n\n"+
            "Please connect to a remote machine before running this script.")
        return
 
    tab.Screen.Synchronous = True
    count = 0
    while count < 2000 / 5:
        tab.Screen.Send('x2dbg')
        crt.Sleep(5)
        count += 1
#---------------------------main--------------------------------                    
Main()