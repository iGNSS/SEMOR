# SEMOR
File di configurazione usati per debug:
- rtk4pid.conf
- ppp4pid_navcast.conf

Il programma avvia:
- un'istanza di str2str che prende come input i dati che arrivano dalla porta 8081 e manda output sulle porte 8085 (per rtk4pid) e 8086 (per ppp4pid_navcast)
- 2 istanze di rtkrcv prendono i dati dalle porte 8085 e 8086 e mandano l'ouput rispettivamente sulle porte 8090 e 8091


Ai 3 processi precedenti viene chiuso input e output da terminale poiché non necessario se non inutile per SEMOR.

I 3 processi verranno chiusi al termine di SEMOR (inviando 'q' o 'Q') o in caso di errore (gestito nel codice) tramite la system call kill().

Gli errori gestiti dal codice conterranno la parola "SEMOR", per tutti gli altri errori bisogna terminare manualmente i 3 processi di RTKLIB (vedi la riga sotto).

Se SEMOR dovesse arrestarsi in modo anonimo per un errore non gestito nel codice, è possibile terminare manualmente i processi utilizzando il comando "kill <pid>", dandogli come input i pid che si trovano nel file pids.txt nella directory root del progetto.

I dati possono arrivare sfasati di un bel po' di secondi, quindi si mantiene una storica dei dati ricevuti con MAX_HISTORY (default 30) elementi (sia per GPS che per GALILEO)

Per debug, il risultato dell'elaborazione è scritto sul file output.txt.

