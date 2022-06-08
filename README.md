### Importante per SiC
Se get_data() ritorna una soluzione  con dLat uguale a -1 allora c'è un mancato posizionamento per quell'epoca. Quindi ignorare/skippare quell'output.



### Setup SEMOR

For now the software requires the absolute path to the SEMOR folder.

In order to change this path you must:
- note the absolute path of the SEMOR folder
- open the semor.c file in the SEMOR folder and modify every "/home/semor/SEMOR/" to the path of your SEMOR folder
- do the same for the client.c inside the src folder

### Compile
You need to compile 3 files: semor.c, rtkrcv.c and str2str.c as shown below.
Commands to be executed inside the SEMOR folder:
```
  make
  cd RTKLIB-b34e/app/consapp/rtkrcv/gcc/
  make
  cd ../../../../..
  cd RTKLIB-b34e/app/consapp/str2str/gcc/
  make
```
### Execute
Commands to be executed inside the SEMOR folder:
```
  cd bin
  ./semor
```
  
In order to stop SEMOR you need to send 'q' to the terminal.
  
The output is written in the output.txt file inside the root folder (SEMOR).

The output is not written immediately, SEMOR have to wait for input from rtkrcv instances.
