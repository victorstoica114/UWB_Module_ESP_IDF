# Carcasă modulară UWB DWM3000 + GPS — prototip V4

V4 păstrează geometria radio/GPS validată în V3 și montează mufa USB-C mamă și
întrerupătorul DPDT într-o nișă retrasă, integrată în laterala carcasei.

Proiectul este construit parametric în CadQuery în jurul modelelor reale:

- `reference_models/modul_radio/UWB.step`;
- `reference_models/antena_gps/YN-91A_refined_v04_assembly.step`;
- `reference_models/acumulator.jpg`.

Fișierele gata de print sunt în `exports`. Pentru imprimare se recomandă
fișierele cu sufixul `_PRINT.stl`; fișierele STEP păstrează geometria CAD.

## Modificările V4

- comenzile nu mai sunt sub carcasă;
- baza cu țintă este un singur disc uniform Ø122 × 5 mm, fără pilotul central
  suprapus;
- o bază alternativă pentru trepied păstrează diametrul de 122 mm și pozițiile
  celor patru magneți, este îngroșată uniform la 8 mm, are chamfer 1 mm pe
  ambele muchii exterioare și filet real 3/8"-16 UNC;
- ținta are patru decupaje străpunse într-o fereastră Ø38 mm, separate de o
  cruce de calibrare de 1 mm pentru poziționare precisă;
- cele patru perechi de magneți au fost mutate de pe raza 19 mm pe raza 44 mm
  și rotite cu 45° pentru stabilitate și degajarea prinderilor interioare;
- USB-C și întrerupătorul sunt stivuite vertical într-o nișă laterală;
- panoul comenzilor este retras cu 12 mm față de marginea nișei;
- în exterior rămâne o proeminență de 3 mm în centru și maximum 7,5 mm la
  marginile ramei;
- plafonul nișei este arcuit pentru imprimare progresivă, fără o consolă
  orizontală mare;
- rama pornește de pe patul de printare și are drenaj inferior;
- mufa USB-C este ghidată și oprită din interiorul corpului;
- toate prinderile care foloseau șuruburi autofiletante au fost convertite la
  M3 cu inserții termice;
- cele două tălpi M3 laterale sunt retrase spre spate și au blocuri de sprijin
  până la podea, flanșe late și nervuri triunghiulare înalte, în stilul
  modificării manuale furnizate;
- o a treia talpă M3, cu două nervuri, leagă centrul peretelui posterior al
  tăvii de un al treilea bosaj din podeaua corpului;
- pragul interior al gulerului și suportul circular al tăvii GPS au tranziții
  autoportante la 45°, pentru a evita suportul continuu pe circumferință;
- suportul GPS are patru brațe și patru distanțiere coaxiale cu găurile reale
  măsurate din STEP: rază 42,5 mm și găuri Ø2,6 mm; fiecare distanțier primește
  o inserție termică M2,5;
- trei degajări radiale rotunjite, cu joc de 0,8 mm, permit suportului GPS să
  coboare drept printre bosajele inserțiilor capacului;
- în jurul celor trei degajări, lobi Ø28 mm reproduc exact ranforsările din
  modelul manual al lui Alex și măresc secțiunea de material a zonelor înguste.

Corpurile rigide ale USB-C și întrerupătorului sunt în interiorul cilindrului.
Numai fețele de acces sunt vizibile în nișă, iar maneta modelată rămâne cu 3 mm
în spatele marginii exterioare.

## Reperul UWB și orientarea componentelor

- placa este verticală, cu DWM3000 sus și mufa SMA jos;
- ESP32 nu este folosit ca reper de poziționare;
- centrul blocului ceramic al antenei DWM3000 este pe X=0, Y=0 față de țintă;
- marginea inferioară a PCB-ului este la 40 mm de podeaua interioară;
- între partea superioară a ansamblului radio și placa „floare” a antenei GPS
  sunt 50 mm;
- antena GPS este orizontală și întoarsă cu fața cerută în sus;
- acumulatorul de 48,5 × 25 × 73 mm este vertical, în spatele plăcii, între
  Z=6,5 și Z=79,5 mm;
- zona mufei SMA are o degajare de 18 × 50 mm, verificată cu un volum de gardă
  Ø15 mm.

## Piese printabile

1. `01_baza_tinta_PRINT.stl` — disc magnetic monolitic Ø122 × 5 mm, cu țintă
   decupată pentru vizualizarea podelei.
2. `02_corp_inferior_PRINT.stl` — corpul cilindric, nișa laterală arcuită,
   ghidajul USB-C, drenajul și locașurile pentru inserții.
3. `03_sasiu_pcb_acumulator_PRINT.stl` — suportul PCB și tava acumulatorului.
4. `04_retainer_acumulator_PRINT.stl` — retainerul acumulatorului.
5. `05_guler_superior_PRINT.stl` — gulerul superior și suportul tăvii GPS.
6. `06_suport_antena_gps_PRINT.stl` — tava antenei GPS cu patru locașuri pentru
   inserții M2,5, patru brațe, distanțiere de 5,07 mm și trei degajări
   ranforsate.
7. `07_capac_palarie_PRINT.stl` — capacul de ploaie.
8. `08_baza_trepied_PRINT.stl` — disc magnetic alternativ Ø122 × 8 mm, cu filet
   elicoidal real 3/8"-16 UNC și locaș pentru flanșă.

Ansamblul complet este:
`exports/UWB_carcasa_modulara_ansamblu_verificare.step`.

## Elemente de asamblare

- 8 magneți disc Ø10 × 2 mm;
- încă 4 magneți disc Ø10 × 2 mm dacă ambele baze interschimbabile sunt
  echipate simultan;
- un șurub sau o inserție metalică pentru trepied cu filet 3/8"-16 UNC,
  lungime de maximum 7 mm și flanșă Ø9,5 × 1,2 mm;
- 9 inserții termice filetate M3, măsurate Ø3,0 mm pe corp, Ø4,0 mm peste
  striații și 5,0 mm lungime;
- 4 inserții termice filetate M2,5, măsurate Ø3,0 mm pe corp, Ø3,5 mm peste
  striații și 5,0 mm lungime;
- 3 șuruburi M3 × 8 mm pentru șasiu;
- 3 șuruburi M3 × 8 mm pentru gulerul superior;
- 3 șuruburi M3 × 8 mm pentru capac;
- 4 șuruburi M2,5 × 6–8 mm pentru antena GPS;
- bandă dublu-adezivă de aproximativ 1 mm pentru placa radio;
- bandă dublu-adezivă pentru baza cu țintă;
- adeziv pentru magneți compatibil cu materialul printat.

Parametrii impliciți pentru prinderi sunt:

- gaură de trecere M3: Ø3,4 mm;
- locaș inserție M3: Ø3,7 × 5,8 mm;
- intrare de centrare M3: Ø4,1 × 0,8 mm;
- locaș inserție M2,5: Ø3,2 × 5,3 mm;
- intrare de centrare M2,5: Ø3,6 × 0,6 mm;
- canal pentru vârful șurubului M2,5: Ø2,8 mm;
- bosaj standard: Ø9 mm;
- locaș cap șurub: Ø6,5 mm.

Pentru baza de trepied, piesa filetată se montează dinspre fața opusă
magneților. Flanșa intră într-un locaș Ø9,8 × 1,35 mm și rămâne la nivelul
feței discului. Filetul din plastic este modelat elicoidal 3/8"-16 UNC, cu pas
1,5875 mm, unghi de 60°, diametru major de degajare Ø9,78 mm și diametru minor
Ø8,08 mm. Lungimea filetată utilă este de 5,80 mm, aproximativ 3,65 spire.
Inserția de 7 mm se oprește cu 1 mm înaintea feței cu magneții.

Dimensiunile inserțiilor diferă între producători. Valorile curente sunt
adaptate măsurătorilor lui Alex; înaintea unui nou lot de inserții se recomandă
o probă și ajustarea parametrilor `m3_insert_*` și `m25_insert_*`.

Găurile antenei GPS sunt Ø2,6 mm în modelul furnizat. M3 nu poate trece prin ele
fără găurirea antenei, de aceea această singură prindere folosește M2,5.
Inserțiile M2,5 se montează de sus în cele patru distanțiere, iar sub fiecare
locaș rămâne un canal Ø2,8 mm pentru vârful șurubului. Centrele suportului coincid
cu centrele STEP cu abatere calculată 0,000 mm.

## Poziția inserțiilor termice

- 3 inserții verticale în bosajele din podeaua corpului, pentru șasiu;
- 3 inserții radiale în corp, pentru guler;
- 3 inserții radiale în guler, pentru capac;
- 4 inserții verticale M2,5 în distanțierele suportului GPS.

Inserțiile trebuie montate înainte de introducerea electronicii. Pentru cele
radiale se recomandă folosirea unui vârf de letcon dedicat, menținut coaxial cu
locașul.

## Ordinea recomandată de montaj

1. Printează o probă pentru magnet, inserție, USB-C și axul întrerupătorului.
2. Montează cele 9 inserții M3 și cele 4 inserții M2,5.
3. Introdu USB-C din interior în ghidajul nișei și fixează întrerupătorul prin
   panoul retras.
4. Lipește placa pe șasiu cu bandă de 1 mm, cu DWM3000 sus și SMA jos.
5. Montează acumulatorul și retainerul, apoi fixează șasiul cu trei M3 × 8 mm.
6. Conectează cablurile, păstrând o rază mare de curbură pentru coaxialul GPS.
7. Coboară suportul GPS drept prin guler, cu cele trei degajări aliniate la
   bosajele capacului, până când se așază pe pragul circular.
8. Fixează antena GPS prin cele patru găuri coaxiale cu șuruburi M2,5.
9. Montează gulerul și capacul cu câte trei șuruburi M3 × 8 mm.
10. Pentru montarea pe trepied, înșurubează piesa 3/8"-16 UNC în baza
    `08_baza_trepied`, dinspre fața opusă magneților, până când flanșa este la
    nivel.
11. Lipește baza cu țintă pe podea sau montează baza pe trepied și cuplează
    magnetic ansamblul.

## Recomandări de printare

- ASA pentru utilizare la exterior; PETG este o alternativă bună;
- duză 0,4 mm, strat 0,20 mm;
- minimum 3 pereți și 4 straturi sus/jos;
- infill 20–30% în zonele cu inserții;
- corpul și gulerul se imprimă vertical;
- nișa arcuită și rama care pornește de la bază reduc necesarul de suport;
- capacul și retainerul sunt deja întoarse în fișierele `_PRINT.stl`;
- gulerul superior se așază cu manșonul lat pe pat; cele două praguri interioare
  au racordări la 45° și păstrează diametrele funcționale;
- șasiul poate necesita brim și suport local.
- baza de trepied se imprimă cu fața locașului flanșei pe pat și locașurile
  magneților în sus; pentru zona centrală se recomandă minimum 5 pereți și
  40% infill;
- filetul 3/8"-16 se imprimă mai curat la strat de 0,16–0,20 mm și viteză mică
  pe pereții interiori; dacă este prea strâns, mărește parametrul
  `tripod_thread_major_d` în pași de 0,10 mm.

## Verificări incluse

`exports/verification.txt` și `exports/manifest.json` confirmă:

- toate cele 8 piese au câte un singur solid BREP valid;
- lipsa interferențelor între PCB, acumulator, GPS și carcasă;
- lipsa interferențelor USB-C/întrerupător cu corpul;
- axa antenei DWM3000 este centrată pe țintă;
- distanțele de 40 mm și 50 mm sunt păstrate;
- antena GPS rămâne rotită cu 180°;
- cele patru găuri GPS sunt pe raza 42,5 mm, cu eroare maximă de centrare
  0,000 mm;
- distanțierele GPS se opresc la 0,10 mm sub placa antenei;
- sunt prevăzute 9 inserții termice M3 și 4 inserții termice M2,5;
- șasiul are trei puncte de prindere: două urechi laterale retrase și
  ranforsate până la podea, plus o ureche posterioară cu două nervuri;
- cele două praguri circulare descendente ale gulerului au tranziții
  autoportante la 45°, cu diametrele de centrare neschimbate;
- suportul GPS rămâne un singur solid, iar coborârea sa axială prin guler,
  verificată din 0,5 în 0,5 mm, are intersecție maximă 0,000 mm³;
- geometria ranforsărilor suportului GPS coincide boolean cu STEP-ul modificat
  manual: diferență 0,000 mm³ în ambele sensuri;
- panoul este retras 12 mm, iar proeminența exterioară maximă este 7,5 mm.
- baza este un disc uniform cu un singur solid, ținta este străpunsă, iar
  buzunarele magneților sunt pe raza 44 mm cu minimum 11,875 mm material până
  la marginea exterioară.
- baza de trepied este un disc uniform Ø122 × 8 mm, folosește aceleași patru
  poziții pentru magneți și păstrează inserția de 7 mm complet între fețe;
- ambele muchii exterioare ale bazei de trepied au chamfer 1 × 1 mm;
- flanșa are 0,30 mm joc diametral și 0,15 mm joc pe adâncime;
- filetul 3/8"-16 este elicoidal, are pas 1,5875 mm, 5,80 mm lungime utilă și
  0,255 mm joc diametral la diametrul major nominal.

## Previzualizări

- `previews/01_ansamblu_exterior.png`;
- `previews/02_ansamblu_transparent.png`;
- `previews/03_piese_explodate.png`;
- `previews/04_aliniere_axa_uwb.png`;
- `previews/05_detaliu_sma_acumulator_coborat.png`;
- `previews/06_baza_tinta_diametru_complet.png`;
- `previews/07_nisa_laterala_comenzi.png`;
- `previews/08_aliniere_gauri_antena_gps.png`;
- `previews/09_talpi_sasiu_ranforsate.png`;
- `previews/10_talpa_dreapta_sasiu_ranforsata.png`;
- `previews/11_baza_tinta_decupata_vedere_sus.png`;
- `previews/12_prinderi_sasiu_trei_puncte.png`;
- `previews/13_racordari_interioare_fdm_45deg.png`;
- `previews/14_trecere_suport_gps_printre_bosaje.png`;
- `previews/15_ranforsari_decupaje_suport_gps.png`;
- `previews/16_baza_trepied_filet_3_8.png`;
- `previews/17_baza_trepied_fata_magneti.png`;
- `previews/18_filet_trepied_sectionat.png`.

Carcasa folosește suprapuneri și protecții geometrice împotriva ploii, dar nu
este etanșă la imersie și nu are un grad IP certificat.

## Regenerare

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe .\generate_enclosure.py
.\.venv\Scripts\python.exe .\render_previews.py
```

Toate cotele importante sunt centralizate în clasa `P` din
`generate_enclosure.py`.
