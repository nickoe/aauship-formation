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
		. Bestil manglende dele
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



Milestones:

- ROS inlejret program på båden der kører
- Autonom styring af båden:
	- Simuleging
	- Hardware: Gps, andet?
- Gruooekoordineringsopgave
- Formationsfølgeropgave
- Avoidance opgave
- Litteratur resheach
	- Undersøgelse af formationsparadigmer
- Fuld dynamisk model af båden incl. forstyrrelser
- Udvidelse af flåden
- Implementering
	- Gruppe
	- Formation
	- Avoidance
