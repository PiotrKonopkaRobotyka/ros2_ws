## Konwencje

- **„Default”** – wartość domyślna z dokumentacji Nav2.
- **„My value”** – wartość z przewodnika, czyli realnie zmieniona / zalecana.
- Dla uproszczenia pomijane są parametry, gdzie `My value == Default` i autor wyraźnie zaleca zostawienie defaultu.
- Tam, gdzie default nie jest podany wprost, opisane jest **znaczenie parametru oraz sposób jego tuningu**, a wartość z przewodnika traktowana jest jako rekomendacja wyjściowa.

Oznaczenia w tekście:
- 🔧 – parametr typowo do tuningu
- ⚠️ – parametr mocno wpływający na stabilność / bezpieczeństwo
- 🧪 – parametr do eksperymentów (zaawansowane przypadki)

***

## AMCL – zmienione parametry

### 🔧 robot_model_type

- **My value:** `nav2_amcl::OmniMotionModel`
- **Default:** `nav2_amcl::DifferentialMotionModel`

**Co robi:**
- Wybiera **model ruchu** używany w filtrze cząsteczkowym (jak AMCL przewiduje przemieszczenie cząsteczek z odometrii).
- Dla robota **omnidirectional (mecanum, omni‑koła)** konieczny jest model, który uwzględnia ruch boczny (`OmniMotionModel`).

**Jak tuningować:**
- Dla robota różnicowego – **zostań przy `DifferentialMotionModel`**.
- Dla omni – **obowiązkowo** przełącz na `OmniMotionModel`; zły model daje systematyczne błędy lokalizacji, szczególnie przy ruchu poprzecznym.
- Jeżeli podczas ruchów bocznych chmura cząsteczek „rozjeżdża się” albo mapa/laser nie pasują wizualnie, sprawdź w pierwszej kolejności właśnie ten parametr.

### 🔧 update_min_a

- **Default:** `0.2` rad (~11.5°)
- **My value:** `0.05` rad (~2.9°)

**Co robi:**
- Minimalny **obrót robota** wymagany, żeby AMCL wykonał aktualizację filtra.

**Skutki tuningu:**
- Większa wartość → rzadsze aktualizacje, mniejsze obciążenie CPU, ale gorsza responsywność na małe obroty (robot może „gubić się” przy bardzo powolnych rotacjach).
- Mniejsza wartość → częstsze aktualizacje, lepsza dokładność przy wolnych manewrach kosztem CPU.

**Rekomendacje:**
- **Powolne, precyzyjne roboty indoor:** wartości rzędu `0.03–0.08` rad.
- **Szybsze platformy / mniej krytyczna dokładność:** bliżej defaultu `0.2`.
- Jeżeli **TF map→odom „drży”** lub lokalizacja „skacze” przy delikatnym ruszaniu joystickiem, warto **zmniejszyć** `update_min_a`.

### 🔧 update_min_d

- **Default:** `0.25` m
- **My value:** `0.05` m

**Co robi:**
- Minimalne **przemieszczenie liniowe**, po którym AMCL wykonuje aktualizację.

**Skutki tuningu:**
- Większa wartość → mniej aktualizacji, dobre dla szybkich robotów w dużych przestrzeniach, ale słabe dla powolnego, precyzyjnego ruchu.
- Mniejsza wartość → lepsza dokładność przy jazdach z małą prędkością, szczególnie w wąskich korytarzach i przy dokowaniu, kosztem CPU.

**Rekomendacje:**
- Dla **robota wolnego, jeżdżącego między drzwiami/regałami** – okolice `0.03–0.07` m.
- Dla **robota magazynowego** z długimi przebiegami – można podnieść do `0.15–0.25` m.

### 🔧 z_short

- **Default:** `0.005`
- **My value:** `0.05`

**Co robi:**
- Waga składowej **`z_short`** w modelu pomiarowym sensora – opisuje **nieoczekiwane krótkie odczyty** (np. słupek, noga krzesła, błyszcząca powierzchnia).

**Skutki tuningu:**
- Większa wartość → AMCL **bardziej ufa krótkim pomiarom** niż wynika to z mapy; lepiej radzi sobie z małymi, nieobecnymi na mapie przeszkodami, ale może być bardziej podatny na szum.
- Mniejsza wartość → krótkie pomiary są uznawane za bardziej „losowe” i mniej wpływają na estymatę pozycji.

**Rekomendacje:**
- W środowiskach z **dużą ilością małych obiektów** (krzesła, nogi stołów, ludzie) – **podniesienie** w okolice `0.02–0.05` ma sens.
- Jeżeli lokalizacja staje się niestabilna w bardzo zaszumionym środowisku, można **zmniejszyć** `z_short` bliżej defaultu.

***

## BT Navigator – zmienione parametry

### 🔧 odom_topic

- **Default:** `"odom"`
- **My value:** `"/odometry/filtered"`

**Co robi:**
- Określa, z którego topica BT Navigator pobiera **odometrię** do podejmowania decyzji.

**Skutki tuningu:**
- Przełączenie na `odometry/filtered` oznacza użycie **odometrii po EKF** (np. z `robot_localization`), co zwykle daje znacznie stabilniejsze dane niż surowe enkodery/IMU.

**Rekomendacje:**
- Jeżeli masz **EKF/UKF** – warto spójnie używać `/odometry/filtered` we wszystkich komponentach (BT, velocity_smoother, itp.).
- Jeżeli robot „szarpie” ruchami z powodu niestabilnej odometrii, upewnij się, że BT i inne serwery korzystają z **tej samej, filtrowanej ramki**.

***

## Controller Server + MPPI + Regulated Pure Pursuit

### Parametry ogólne Controller Server

#### 🔧 controller_frequency

- **Default:** `20.0` Hz
- **My value:** `5.0` Hz

**Co robi:**
- Częstotliwość generowania komend prędkości przez kontroler lokalny.

**Skutki tuningu:**
- Niższa wartość → mniejsze obciążenie CPU, ale mniej „gęste” sterowanie (bardziej schodkowe trajektorie).
- Wyższa wartość → bardziej płynne śledzenie ścieżki kosztem CPU.

**Rekomendacje:**
- Dla **PC klasy desktop / NUC** i niewielkiej liczby robotów – 15–20 Hz.
- Dla **słabszych SBC** (Pi‑class) można zejść do 5–10 Hz, tak jak w przewodniku.

#### 🔧 min_x/y/theta_velocity_threshold

- **Default:** `0.0001` (x, y, theta)
- **My value:** `0.001` (x, y, theta; y domyślnie było dobrane striczej dla diff‑drive)

**Co robi:**
- Minimalne wartości prędkości, poniżej których ruch traktowany jest jak **szum** i ignorowany.

**Skutki tuningu:**
- Zbyt małe progi → „pływająca” odometria, wiele bardzo małych komend, mogących powodować drgania.
- Zbyt duże progi → robot nie reaguje na bardzo małe ruchy (przydaje się jednak do filtrowania szumu enkoderów).

**Rekomendacje:**
- Zostaw rząd wielkości `1e‑3` dla robotów indoor.
- Dla bardzo precyzyjnych, wolnych manipulatorów mobilnych można zejść niżej, ale zawsze obserwuj **czy nie pojawia się drganie na miejscu**.

#### 🔧 failure_tolerance

- **Default:** `0.0` s
- **My value:** `0.3` s

**Co robi:**
- Czas, przez jaki kontroler **może chwilowo nie generować poprawnych komend**, zanim akcja `FollowPath` zostanie uznana za nieudaną.

**Skutki tuningu:**
- Większa tolerancja → większa odporność na krótkie dropy sensoryczne / chwilowe braki ścieżki, ale dłużej czekasz, zanim system uzna, że coś jest ewidentnie nie tak.
- `0.0` → każde chwilowe „potknięcie” kończy się błędem.

**Rekomendacje:**
- 0.2–0.5 s sprawdza się w większości systemów indoor.
- Dla robotów w **mocno niestabilnym środowisku sieciowym** można pójść wyżej.

#### 🔧 goal_checker_plugins + tolerancje

- **Default plugins:** `["goal_checker"]`
- **My value:** `["general_goal_checker"]`
- **general_goal_checker.xy_goal_tolerance:** `0.35` (default `0.25`)
- **general_goal_checker.yaw_goal_tolerance:** `0.50` rad (default `0.25`)

**Co robi:**
- Definiuje, **kiedy cel uznajemy za osiągnięty** (promień i tolerancja orientacji).

**Skutki tuningu:**
- Większe tolerancje → mniej „tańczenia” przy celu, większa robustność, ale mniejsza precyzja pozycji.
- Mniejsze tolerancje → lepsze dokowanie kosztem ryzyka, że robot będzie długo „kręcił się” obok celu.

**Rekomendacje:**
- Do zwykłego **„podjedź w okolice drzwi”** – spokojnie `0.3–0.4` m i `0.4–0.6` rad.
- Do **dokowania / ładowarki** – zejść nawet do `0.05–0.1` m i `0.1–0.2` rad, ale wymaga bardzo dobrze zestrojonej kinematyki i kontrolera.

### Zmiana pluginu FollowPath

#### 🔧 FollowPath.plugin

- **Default:** `dwb_core::DWBLocalPlanner`
- **My value:**
  - `nav2_mppi_controller::MPPIController` **lub**
  - `nav2_rotation_shim_controller::RotationShimController` z `RegulatedPurePursuitController` jako `primary_controller`

**Co robi:**
- Określa **lokalny kontroler** odpowiedzialny za generowanie komend prędkości na podstawie ścieżki.

**Skutki zmiany:**
- **MPPIController** – symuluje wiele trajektorii i wybiera najlepszą; bardzo płynny ruch, dobrze radzi sobie z ograniczeniami dynamicznymi, ale jest **ciężki obliczeniowo**.
- **RotationShimController + RegulatedPurePursuit** – najpierw obrót w stronę ścieżki, potem prostsze śledzenie punktu „carrot”; przewidywalne zachowanie w ciasnych korytarzach, zwykle lżejsze obliczeniowo.

**Rekomendacje:**
- Jeżeli masz **mocny komputer** i zależy Ci na bardzo płynnym ruchu – MPPI.
- Jeżeli priorytetem jest **prostota tuningu i stabilność w drzwiach/korytarzach** – Rotation Shim + Regulated Pure Pursuit.

### MPPI – zmienione parametry

#### 🔧 time_steps

- **Default:** `56`
- **My value:** `15`

**Co robi:**
- Liczba kroków czasowych w horyzoncie predykcji.

**Skutki tuningu:**
- Większa wartość → dłuższy horyzont, lepsze planowanie „w przyszłość”, ale **koszt rośnie liniowo** z liczbą kroków.
- Mniejsza wartość → krótszy horyzont, szybsze obliczenia, gorsza przewidywalność daleko przed robotem.

**Rekomendacje:**
- Dla robotów indoor z prędkościami ~0.5 m/s wartości `10–20` są zazwyczaj wystarczające.
- W bardzo dynamicznych środowiskach lub przy dużych prędkościach można rozważyć zwiększenie, ale trzeba pilnować CPU.

#### 🔧 model_dt

- **Default:** `0.05` s
- **My value:** `0.2` s

**Co robi:**
- Odstęp czasowy między kolejnymi krokami predykcji.

**Skutki tuningu:**
- Większa wartość → rzadziej próbkowany horyzont (grubsza siatka w czasie), mniej obliczeń, ale większe ryzyko „ominięcia” ciasnych przeszkód.
- Mniejsza wartość → dokładniejsze próbkowanie, lepsza jakość trajektorii, większy koszt.

**Rekomendacje:**
- Dobrze jest **zsynchronizować `model_dt` z `controller_frequency`** (np. `controller_frequency = 5 Hz → model_dt = 0.2 s`).
- Jeżeli przechodzisz na wyższe częstotliwości sterowania, rozważ zmniejszenie `model_dt`.

#### 🔧 batch_size

- **Default:** `1000`
- **My value:** `10000`

**Co robi:**
- Liczba trajektorii losowanych w każdym kroku optymalizacji.

**Skutki tuningu:**
- Większa wartość → lepsze pokrycie przestrzeni sterowań, potencjalnie lepsze trajektorie, ale **mocno rosną wymagania CPU/GPU**.
- Mniejsza wartość → szybsze działanie, ale większa podatność na lokalne minima.

**Rekomendacje:**
- Zacznij od 1000–2000 i stopniowo zwiększaj, obserwując obciążenie CPU.
- 10000 ma sens tylko przy mocnej maszynie i gdy całe Nav2 nadal mieści się w budżecie czasu rzeczywistego.

#### 🔧 wz_std

- **Default:** `0.2` rad/s
- **My value:** `0.4` rad/s

**Co robi:**
- Odchylenie standardowe próbkowania prędkości kątowej w trakcie generowania trajektorii.

**Skutki tuningu:**
- Większa wartość → kontroler **próbuje bardziej zróżnicowanych prędkości obrotowych**, lepiej odnajduje „dziury” w przeszkodach, ale może generować bardziej agresywne obroty.
- Mniejsza wartość → „grzeczniejsze” skręty, kosztem mniejszej elastyczności.

**Rekomendacje:**
- Dla wąskich korytarzy i wielu zakrętów (np. biuro) warto mieć wyższą `wz_std`.
- Jeżeli robot zaczyna wykonywać zbyt gwałtowne ruchy, zmniejsz wartość.

#### 🔧 vx_min

- **Default:** `-0.35` m/s (dopuszcza jazdę do tyłu)
- **My value:** `0.0` m/s

**Co robi:**
- Minimalna prędkość liniowa w osi x (do przodu/tyłu).

**Skutki tuningu:**
- Ustawienie `0.0` **wyłącza jazdę wstecz** – robot będzie zawsze starał się jechać do przodu (ewentualnie obracając się).

**Rekomendacje:**
- W ciasnych przestrzeniach z ludźmi często lepiej **wyłączyć ruch do tyłu** (bezpieczeństwo i przewidywalność).
- W magazynach, gdzie wsteczny jest pożądany, zostaw wartość ujemną.

### Krytyki MPPI – najważniejsze zmiany

#### ⚠️ CostCritic – consider_footprint

- **Default:** `consider_footprint = false`
- **My value:** `consider_footprint = true` (dodatkowo `near_goal_distance = 1.0`, `trajectory_point_step = 2`)

**Co robi:**
- Używa **rzeczywistego obrysu robota** (footprint) przy ocenie kosztu przeszkód zamiast traktowania robota jako punktu.

**Skutki tuningu:**
- `true` → **realistyczne marginesy bezpieczeństwa**, lepsze omijanie narożników, większy koszt obliczeniowy.

**Rekomendacje:**
- Dla robotów, które mają wyraźnie podłużną/niestandardową sylwetkę, **warto włączyć**.
- Jeżeli CPU jest na granicy, można przetestować wyłączenie, ale kosztem bezpieczeństwa.

#### 🔧 PathAlignCritic.cost_weight

- **Default:** `10.0`
- **My value:** `14.0`

**Co robi:**
- Wymusza **trzymanie się ścieżki** (kara za odchylenie od planu).

**Rekomendacje:**
- Podnoszenie w okolice `12–16` ma sens, gdy planner globalny daje dobre ścieżki i chcesz, żeby lokalny kontroler ich „nie skracał” przy ścianach.
- Zbyt duża waga może jednak utrudnić omijanie nagłych przeszkód.

#### 🔧 PathAngleCritic.cost_weight

- **Default:** `2.2`
- **My value:** `2.0`

**Co robi:**
- Kara za niezgodność **orientacji trajektorii** z kierunkiem ścieżki.

**Rekomendacje:**
- Delikatne zmniejszenie (jak w przewodniku) pozwala na trochę luźniejszą orientację przy zachowaniu dobrej zgodności pozycji.

***

## Regulated Pure Pursuit – zmienione parametry

### 🔧 desired_linear_vel

- **Default:** `0.5` m/s
- **My value:** `0.4` m/s

**Co robi:**
- Docelowa prędkość liniowa na odcinkach prostych.

**Rekomendacje:**
- Dla wnętrz z ludźmi lepiej `0.3–0.5` m/s niż wyższe wartości.
- Zwiększaj tylko, jeżeli robot mechanicznie jest stabilny i ścieżki są „gładkie”.

### 🔧 lookahead_dist / min_lookahead_dist / max_lookahead_dist

- **Default:** `0.6 / 0.3 / 0.9` m
- **My value:** `0.7 / 0.5 / 0.7` m

**Co robi:**
- Określa, jak daleko wzdłuż ścieżki wybierany jest **punkt „carrot”**, do którego steruje regulator.

**Skutki tuningu:**
- Większy lookahead → **płynniejsze, szersze łuki**, ale może „ścinać” zakręty.
- Mniejszy → bardziej agresywne podążanie za ścieżką, możliwość szarpnięć.

**Rekomendacje:**
- Dla wózka wielkości człowieka w biurze typowo `0.5–0.8` m.
- Minimalny lookahead nie powinien być zbyt mały, bo wtedy robot przy małej prędkości zaczyna „mydlić” tor.

### 🔧 rotate_to_heading_angular_vel

- **Default:** `1.8` rad/s
- **My value:** `0.375` rad/s

**Co robi:**
- Prędkość obrotu podczas fazy „rotate to heading”.

**Skutki tuningu:**
- Niższa wartość → spokojne, przewidywalne obroty, ważne przy ciężkich robotach lub śliskiej posadzce.

**Rekomendacje:**
- Dla robotów w otoczeniu ludzi i przy ograniczonej przyczepności lepiej pozostać przy **0.3–0.6 rad/s** niż przy domyślnych wartościach bliskich 2 rad/s.

### 🔧 use_velocity_scaled_lookahead_dist

- **Default:** `false`
- **My value:** `true`

**Co robi:**
- Skaluje **lookahead** w zależności od prędkości – przy większej prędkości robot patrzy dalej.

**Rekomendacje:**
- W praktyce niemal zawsze **warto to włączyć**, bo zachowanie staje się naturalniejsze (jak człowiek patrzący dalej przy szybszym marszu).

### 🔧 approach_velocity_scaling_dist

- **Default:** `1.0` m
- **My value:** `0.6` m

**Co robi:**
- Odległość od celu, od której zaczyna się **wyhamowywanie**.

**Skutki tuningu:**
- Mniejsza odległość → robot zwalnia później; ruch bardziej dynamiczny, ale może „wjeżdżać” w cel.
- Większa → łagodniejsze dojazdy.

**Rekomendacje:**
- Dla niewielkich prędkości 0.4–0.7 m zwykle wystarcza.
- Dla ciężkich platform lub przy wyższych prędkościach warto **zwiększyć**, nie zmniejszać.

### 🔧 regulated_linear_scaling_min_radius

- **Default:** `0.9` m
- **My value:** `0.85` m

**Co robi:**
- Minimalny promień wokół przeszkód, w którym zaczyna działać **skalowanie prędkości liniowej**.

**Rekomendacje:**
- Drobne zmniejszenie (0.8–0.9) pozwala **nieco ciaśniej** mijać przeszkody bez przesadnego zwalniania.

### 🔧 curvature_lookahead_dist

- **Default:** `1.0` m
- **My value:** `0.6` m (przy `use_fixed_curvature_lookahead=false`)

**Co robi:**
- Dystans używany do oceny krzywizny ścieżki.

**Rekomendacje:**
- Mniejsza wartość poprawia zachowanie na ciasnych zakrętach.

### 🔧 max_allowed_time_to_collision_up_to_carrot

- **Default:** `1.0` s
- **My value:** `1.5` s

**Co robi:**
- Minimalny dozwolony czas do kolizji na odcinku do punktu „carrot”.

**Rekomendacje:**
- Wyższa wartość (1.5–2.0 s) → **bardziej zachowawcza** jazda; dobra dla środowisk z ludźmi.

***

## Local costmap – zmienione parametry

### Ramki, rozdzielczość, okno

#### 🔧 global_frame

- **Default:** `"map"`
- **My value:** `"odom"`

**Co robi:**
- Ramka odniesienia lokalnej costmapy.

**Rekomendacje:**
- Typowy pattern: **global_costmap w "map"**, local_costmap w `"odom"` – dokładnie jak w przewodniku.

#### 🔧 rolling_window

- **Default:** `false`
- **My value:** `true`

**Co robi:**
- Ustawia costmapę jako „ruchome okno” wokół robota.

**Rekomendacje:**
- Dla lokalnej costmapy **zawsze włączone** (`true`) – inaczej traci sens.

#### 🔧 resolution

- **Default:** `0.1` m/komórkę
- **My value:** `0.05` m/komórkę

**Co robi:**
- Rozdzielczość siatki costmapy.

**Skutki tuningu:**
- Mniejsza komórka → **dokładniejsza mapa**, lepsze odzwierciedlenie wąskich przesmyków, ale koszt CPU i pamięci rośnie ~4× przy zmianie 0.1→0.05.

**Rekomendacje:**
- Indoor, drzwi ~80–90 cm, robot ~40–60 cm – 0.05 m jest rozsądne.
- Jeżeli CPU jest za słaby, można wrócić do 0.1 m.

#### 🔧 robot_radius

- **Default:** `0.1` m
- **My value:** `0.15` m

**Co robi:**
- Promień modelu robota używany przy inflacji i footprint‑based collision checking.

**Rekomendacje:**
- Ustaw **rzeczywisty pół‑przekrój** robota z marginesem bezpieczeństwa (zazwyczaj 2–5 cm).

### Obstacle layer (LIDAR)

#### 🔧 observation_sources, scan.topic i zakresy

- **My value:** `observation_sources = scan`, `scan.topic = /scan`
- `scan.raytrace_min_range = 0.20` m (default `0.0`)
- `scan.obstacle_min_range = 0.20` m (default `0.0`)
- `scan.max_obstacle_height = 2.0` m (default `0.0`)
- `scan.clearing = true` (default `false`)

**Co robi:**
- Ustawia źródło danych dla warstwy przeszkód (LIDAR) i to, **od jakiej odległości** są one brane pod uwagę.

**Rekomendacje:**
- Minimum 0.15–0.25 m blisko LIDAR‑a, jeśli część robota jest w tym obszarze – unikniesz „widzenia własnego ciała”.
- `max_obstacle_height` dopasuj do wysokości ludzi/mebli (typowo 1.5–2.0 m).
- Zawsze włącz `clearing = true` dla dynamicznego środowiska.

### Voxel layer (kamera głębi)

Kluczowe zmiany:
- `z_voxels = 16` (default `10`)
- `observation_sources = realsense1`
- `realsense1.topic = /cam_1/depth/color/points`
- `realsense1.max_obstacle_height = 2.0` (default `0.0`)
- `realsense1.obstacle_max_range = 1.25` (default `2.5`)
- `realsense1.obstacle_min_range = 0.05` (default `0.0`)
- `realsense1.raytrace_min_range = 0.05` (default `0.0`)
- `realsense1.data_type = PointCloud2` (default `LaserScan`)

**Rekomendacje do tuningu:**
- Zakres `obstacle_max_range` dostosuj do **realnego, wiarygodnego zasięgu** kamery (dla D435 sensowne 1–2 m).
- Liczbę warstw `z_voxels` dobierz do tego, czy chcesz wykrywać obiekty nad głową – dla indoor 10–16 zwykle wystarcza.

### Range sensor layer

- **enabled = false** zamiast default `true`
- reszta parametrów ustawiona pod `/ultrasonic1`, ale nieaktywna

**Strategia:**
- Warstwę można **włączyć później**, jeżeli dołożysz realne sensory odległości.

### 🔧 Denoise layer

- **enabled = true` (zgodnie z defaultem), ale świadomie używana
- `minimal_group_size = 2`
- `group_connectivity_type = 8`

**Co robi:**
- Usuwa pojedyncze „piksele hałasu” z mapy (izolowane komórki przeszkód).

**Rekomendacje:**
- Z `minimal_group_size = 2` usuwane są pojedyncze punkty – dobry kompromis.
- Przy bardzo zaszumionym LIDAR‑ze można tę wartość podnieść, ale **ostrożnie**, żeby nie kasować realnych cienkich przeszkód.

### ⚠️ Inflation layer (local)

- **inflation_radius:** `1.75` m (default `0.55`)
- **cost_scaling_factor:** `2.58` (default `1.0`)

**Co robi:**
- Tworzy bufor bezpieczeństwa wokół przeszkód.

**Skutki tuningu:**
- Duży `inflation_radius` + większy `cost_scaling_factor` → robot **trzyma się ładnie środka korytarza** i nie ociera się o ściany, ale może mieć problem z przejechaniem w bardzo wąskich gardłach.

**Rekomendacje:**
- 1.5–2.0 m sprawdza się dobrze dla korytarzy biurowych.
- Jeżeli robot **odmawia przeciśnięcia się** między regałami, zmniejsz radius albo skalę.

***

## Global costmap – zmienione parametry

### 🔧 publish_frequency

- **Default:** `1.0` Hz
- **My value:** `5.0` Hz

**Co robi:**
- Częstotliwość publikowania globalnej costmapy do innych nodów.

**Rekomendacje:**
- 3–5 Hz jest rozsądne w dynamicznym indoorze; 1 Hz wystarczy w bardzo statycznych mapach.

### 🔧 resolution, robot_radius

- Analogicznie do local_costmap: `resolution = 0.05` m, `robot_radius = 0.15` m (podniesione względem defaultu).

### 🔧 track_unknown_space

- **Default:** `false`
- **My value:** `true`

**Co robi:**
- Rozróżnia **obszar nieznany** od wolnego w globalnej mapie.

**Rekomendacje:**
- Dla robotów eksplorujących nowe obszary – zdecydowanie `true`.

### 🔧 plugins

- **Default:** `[static_layer, obstacle_layer, inflation_layer]`
- **My value:** `[static_layer, obstacle_layer, voxel_layer, range_sensor_layer, inflation_layer]` (range_sensor_layer wyłączony)

**Co robi:**
- Dodaje voxele 3D i potencjalnie dane z sensorów zasięgu również do globalnej mapy.

**Rekomendacje:**
- Jeżeli masz tylko 2D LIDAR – możesz pominąć `voxel_layer`.
- Jeżeli masz kamerę głębi – pattern z przewodnika jest sensowny.

### Obstacle layer (global) – jak w local

Te same modyfikacje co w local_costmap:
- `scan.raytrace_min_range = 0.20`
- `scan.obstacle_min_range = 0.20`
- `scan.max_obstacle_height = 2.0`
- `scan.clearing = true`
- `scan.topic = /scan`

### Voxel layer (global)

Zmiany podobne do lokalnej voxeli (inne nazwy źródła):
- `observation_sources = robot_depth_camera`
- `robot_depth_camera.topic = /rgbd_camera`
- zakresy i progi odległości/ wysokości dostosowane do D435.

### ⚠️ Inflation layer (global)

- **inflation_radius:** `1.75` m
- **cost_scaling_factor:** `2.58` – identycznie jak w local_costmap.

**Rekomendacje:**
- Dla spójności trzymaj te wartości **takie same w global i local**, chyba że masz bardzo szczególny przypadek.

***

## Map saver – zmienione parametry

### 🔧 save_map_timeout

- **Default:** `2.0` s
- **My value:** `5.0` s

**Co robi:**
- Maksymalny czas na zapis mapy.

**Rekomendacje:**
- Dla dużych map rasterowych 5–10 s jest rozsądne.

***

## Planner server – zmienione parametry

### GridBased.plugin – nazwa

- **Default:** `nav2_navfn_planner/NavfnPlanner`
- **My value:** `nav2_navfn_planner::NavfnPlanner`

To jedynie **zmiana składni nazwy pluginu** (nowy styl `::`), funkcjonalnie to samo.

***

## Smoother server – zmienione parametry

### 🔧 simple_smoother.max_its (brak defaultu w guide)

- **My value:** `1000`

**Co robi:**
- Maksymalna liczba iteracji algorytmu wygładzania.

**Jak tuningować:**
- Jeżeli ścieżki są krótkie i proste – wystarczy 100–300.
- 1000 daje duży margines, zwykle bez zauważalnego wpływu na czas (bo iteracje kończą się wcześniej, po spełnieniu tolerancji).

***

## Behavior server – zmienione parametry

### 🔧 behavior_plugins

- **Default:** `["spin", "backup", "drive_on_heading", "wait"]`
- **My value:** `["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]`

Dodanie `assisted_teleop` – **pół‑ręczne sterowanie z zabezpieczeniem przed kolizją**.

### 🔧 max_rotational_vel

- **Default:** `1.0` rad/s
- **My value:** `0.5` rad/s

Bezpieczniejsze, wolniejsze rotacje dla zachowań recovery.

### 🔧 enable_stamped_cmd_vel

- **Default (dla nowszych Nav2):** `true`
- **My value:** `false` (używa `geometry_msgs/Twist`, nie `TwistStamped`)

**Rekomendacja:**
- Upewnij się, że jest to **spójne z resztą pipeline’u** (velocity_smoother, kontroler) – albo wszystkie moduły używają wersji timestamped, albo żaden.

***

## Waypoint follower – zmienione parametry

### 🔧 loop_rate

- **Default:** `20` Hz
- **My value:** `2` Hz

Niższy loop rate wystarcza do sprawdzania statusu nawigacji między waypointami i zmniejsza obciążenie CPU.

### 🔧 stop_on_failure

- **Default:** `true`
- **My value:** `false`

Robot **kontynuuje do kolejnych waypointów**, nawet jeśli jeden z nich nie został osiągnięty.

### Parametry wait_at_waypoint (brak defaultów w guide)

- `wait_at_waypoint.enabled = true`
- `wait_at_waypoint.waypoint_pause_duration = 10` s

**Co robią:**
- Włączają plugin czekający określony czas na każdym waypoint’cie – np. na stabilizację sensora czy wykonanie zadania.

**Tuning:**
- Czas pauzy zależy od aplikacji; 5–10 s to dobry start.

***

## Velocity smoother – zmienione parametry

### 🔧 max_velocity / min_velocity (oś y)

- **Default:** `[0.5, 0.0, 2.5]` / `[-0.5, 0.0, -2.5]`
- **My value:** `[0.5, 0.5, 2.5]` / `[-0.5, -0.5, -2.5]`

**Co robi:**
- Pozwala na **ruch lateralny** (±0.5 m/s) – ustawienie pod robota omnidirectional.

### 🔧 max_accel / max_decel (x, y)

- **Default:** `[2.5, 0.0, 3.2]` / `[-2.5, 0.0, -3.2]`
- **My value:** `[0.3, 0.3, 3.2]` / `[-0.5, -0.5, -3.2]`

**Co robi:**
- Ogranicza przyspieszenia i hamowania, zapewniając **łagodne, nie‑szarpiące ruchy**.

**Rekomendacje:**
- Dla robotów indoor z przekładniami, które nie lubią szarpnięć, 0.2–0.5 m/s² w x,y jest rozsądne.

### 🔧 odom_topic

- **Default:** `odom`
- **My value:** `odometry/filtered`

Spójnie z BT Navigator – smoother patrzy na **filtrowaną odometrię**.

### 🔧 enable_stamped_cmd_vel

- Analogicznie do BehaviorServer ustawione na `false`.

***

## Collision monitor – zmienione parametry

### 🔧 transform_tolerance

- **Default:** `0.1` s
- **My value:** `0.2` s

Większa tolerancja na opóźnienia TF – pomocne na wolniejszych komputerach.

### 🔧 source_timeout

- **Default:** `2.0` s
- **My value:** `1.0` s

Bardziej rygorystyczne podejście: dane sensora starsze niż 1 s są uznawane za nieaktualne.

### 🔧 state_topic

- **Default:** pusty
- **My value:** `collision_monitor_state`

Umożliwia monitorowanie aktywnych stref bezpieczeństwa i stanów monitora (debug, diagnostyka).

***

## Podsumowanie strategii tuningu

1. **Zacznij od zestawu z przewodnika** – wartości dobrane są do indoorowego robota omni z LIDAR‑em i RealSense.
2. Tuning prowadź **modułami**:
   - najpierw **AMCL + odometria**, aż TF jest stabilny,
   - potem **costmapy (resolution, inflation, obstacle/voxel layers)**,
   - na końcu **kontroler (MPPI/RPP, krytyki, velocity_smoother)**.
3. Zmieniaj **jeden parametr naraz** i zapisuj efekty.
4. Dla bezpieczeństwa priorytet mają:
   - `inflation_radius`, `cost_scaling_factor`,
   - limity prędkości i przyspieszeń,
   - konfiguracja collision monitor i behavior server.
5. Zawsze testuj nowe ustawienia **najpierw w symulacji** (Gazebo / Isaac / Stage), a dopiero potem na realnym robocie.


