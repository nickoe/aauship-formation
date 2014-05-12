TODO listen over alle at-gøre-lister
====================================

Overordnet mål
--------------
Formationskontrol med mindst to aauships
	- To måder: indivuduelle paths eller formation?
	- Det skal funke til sommer!
	- Bestem os for hvad for noget formation vi vil lave, læs papers igennem (formation eller individuelle paths)

Opgavelisten
------------
* Generer en path (NEDPRIORITERET) [Brug manuelle arrays med paths for nu]
* Få 'mariner.m' til at reagere på input OK
* Få mariner til at følge en path OK
* Heading autopilot til mariner OK
* Fix rælæafbrydere med nøgle
* Fix rqt GUI
  - Print data op gui
* Lav ny battery monitor print
* Lav software til battery monitor
* Få autopiloten ind i sin egen funktion i stedet for kode i main.m
* Heading autopilot til Twin Propeller Ship aauship.m
* Få flere mariner'ere til at følge en path OK
* Opdater mariner.m til at være AAUSHIP (modellering) [se chap 2 og 3 i fossen] OK
* Implementer path-follower på AAUSHIP (python)
* Tilføj flere AAUSHIPSs til flåden (praktisk ting)
* Få dem til at danne formation (group coordination task)
* Få formationen til at følge en path (formation mission task)


Andre ting der skal kigges på i ny og næ
----------------------------------------
For at få en funkende båd er der en række praktiske gøremål, der
berører:	

* Software
	- GRS
	- HLI
	- LLI
* Hardware
	- Knapafbryder i stedet for klemmerækken (praktisk ting)
	- Byg to både mere
		. Bestil manglende dele OK
		. Sæt delene sammen
* Dokumentation
	- Basal ledningsføringsdokumentation [nickoe]
	- Ledingsnet
	- Batterihåndtering
	- Hold grabberne væk hvis du er en spasser

Senere todo, når båden er sejlklar
----------------------------------
* Undersøg mere RTK GPS, det vi har nu er ikke testet vældigt godt under bevægelse.
* Få styr på LLI sw og HLI sw
* Mere SW
	- Implementer software til battery monitoren

Fuld manuel kontrol giver også angrebspunkt
Restoring forece mangler i B.15 og så videre, altså for at få anden
ordens differentialligning for for metode 2.
Bollard pull test på hver propel

Milepæle
--------

- ROS inlejret program på båden der kører
- Autonom styring af båden:
	- Simuleging
	- Hardware: Gps, andet?
- Gruppekoordineringsopgave
- Formationsfølgeropgave
- Eventuel undvigelsesopgave
- Litteratur research
	- Undersøgelse af formationsparadigmer
- Fuld dynamisk model af båden incl. forstyrrelser
- Udvidelse af flåden
- Implementering
	- Gruppe
	- Formation
	- Eventuel undvigelse

Opdeling af milepæle:
Datoer i () er til, og ikke med. Der er ikke lagt vægt på så meget til næste semester.
# Dette semester
- (Nu - Slut) Litteratur research
- (Uge 11) Undersøg formationsparadigmer
	Dette er mest en "opvarmning" til at skulle lave formationskontrol.
	Der skal undersøges hvad det kræver at lave
	formationskontrol. Indledning
- (Uge 13) Få enten ROS eller noget selvlavet op at køre på båden
	Systemet på den eksisterende båd skal fungere, så den kan sejle simpelt inden vi indkluderer noget formationskontrol til den. Dermed skal næste punkt også laves. Hvis ROS kan implementeres som et overordnet system og opbygningen i noder kan laves vil dette være fordelagtigt, da andre moduler allerede er under udarbejdning. Derfor er det sådan set kun _infrastruktur_ af ROS. Dette skal også dokumenteres.
- (Uge 14) Deltest af båden, manuel kontrol
	Dette er bare for at teste det simpleste system og funktioner.
- (Uge 18) Dynamisk model af båden incl. forstyrrelser
	En ordentlig model af båden vil være godt, da vi dermed kan styre den mere præcist. Vi regner med at bruge metoden fra _Fossen_, og derfor skal vi have bestemt nogle hydrodynamiske koefficienter ud fra enten nogle nye tests eller dem Lunde og Brian lavede sidste semester.
- (Uge 20) Simpel estimator/determinering
	Det at få nogle states som vi kan bruge til at sejle med
- (Uge 22) Simulering af autonom styring af båden med den nye model
	Når modelleringen er færdig skal båden simuleres i matlab. Måske kan vi også undersøge Gazebo.
- (Uge 23) Test af automon styring til båden
	Når alle forgående punkter er opfyldt skal det modelbaserede til båden implementeres. Herunder angår autonom styring og basal stifølger, skal dette testes. Dette skal skal som det sidste i den første halvdel af projektet.
- (Nu - Uge 23) Undersøg gruppekoordineringsopgave
	Vidensopbygning til næste del af projektet.
- (Nu - Uge 23) Undersøg formationsfølgeropgave
	Vidensopbygning til næste del af projektet.

# Næste semester
- () Få alt hardware op at køre, her tænkes mest på en båd mere
- () Lav og implementer gruppekoordineringsopgave, formationsfølgeropgave og eventuel undvigelsesopgave
- () Få begge både til at sejle i gruppe og hold formation i en given path.
