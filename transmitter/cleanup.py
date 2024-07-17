import os


def clean(numFiles):
    for i in range(numFiles):
        try:
            os.remove('varlist{}'.format(i))
            os.remove('trace{}'.format(i))
            print('Deleted varlist{} and trace{}'.format(i,i))
        except:
            print('No varlist{} and trace{}'.format(i,i))
    
