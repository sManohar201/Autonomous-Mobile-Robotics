# Autonomous Balloon Interception Drone — Project Handoff

آخر تحديث: 2026-09-20
حالة التوجيه: **Thermal + IMU + Classical Computer Vision هو الاتجاه الحالي المعتمد**
حالة YOLO / Deep Learning: **PAUSED — سجل تاريخي فقط**

## قاعدة الاستمرارية

هذا الملف هو نقطة البداية لأي شات جديد. لا تستأنف YOLO26 أو YOLOE أو Det-Fly أو VisDrone أو تدريب النماذج إلا بطلب صريح جديد من صاحب المشروع. المرحلة التنفيذية الحالية تبدأ من إضافة كاميرا حرارية إلى X500.

## مفهوم المشروع

تطوير نظام منخفض التكلفة لطائرة مسيّرة يكتشف أهدافًا عامة ويتتبع عدة أهداف ويلاحق هدفًا مختارًا بصريًا داخل المحاكاة، باستخدام الرؤية الحاسوبية التقليدية فقط. يعتمد النظام الأساسي على كاميرا حرارية واحدة وPX4 / onboard IMU.

لا تحتاج النسخة الأساسية إلى كاميرا RGB، ولا تعتمد على لون الهدف، ولا تستخدم neural networks أو نماذج كشف مدرّبة أو GPU-heavy inference. يمكن بحث RGB لاحقًا كتحسين اختياري.

العنوان المفضل:

> Low-Cost Autonomous Multi-Target Thermal Visual Tracking and Pursuit Using Classical Computer Vision

سؤال البحث:

> Can a low-cost UAV achieve reliable autonomous target detection, multi-target tracking, motion estimation, and closed-loop visual pursuit using deterministic classical computer-vision methods without deep learning?

## معمارية النظام

```text
Thermal Camera + PX4 IMU
          ↓
Thermal Image + Vehicle Motion
          ↓
Adaptive / Dynamic Thermal Thresholding
          ↓
Morphological Filtering
          ↓
Connected Components / Contour Extraction
          ↓
Geometric + Temporal Candidate Filtering
          ↓
Multi-Target Tracking + Persistent Track IDs
          ↓
Kalman Filter + Data Association
          ↓
Short-Horizon Motion Prediction
          ↓
Operator / Deterministic Target Selection
          ↓
Closed-Loop Visual Pursuit Controller
          ↓
PX4 Velocity / Yaw / Trajectory Setpoints
          ↓
Safe Standoff Point while maintaining visual lock
```

## الكشف والتتبع

لا يحتاج النظام إلى تصنيف الهدف دلاليًا كدرون أو بالون أو مركبة. يستخرج object candidates عامة بناءً على التباين الحراري والخصائص الهندسية والحركية والزمنية. يمكن أن يكون الهدف جويًا أو أرضيًا، ثابتًا أو متحركًا، بشرط امتلاكه خصائص حرارية قابلة للرصد.

خصائص التصفية تشمل area، وأبعاد bounding box، وaspect ratio، وperimeter، وcircularity عند فائدتها، وsolidity، وconvexity، واستمرارية الشكل والظهور عبر الزمن.

يحافظ كل مسار على بيانات مثل:

```text
Target ID
center_x, center_y
velocity_x, velocity_y
predicted_center_x, predicted_center_y
tracking confidence / persistence
last_seen_time
track_state
```

استخدم Kalman Filter للتنبؤ القصير، وHungarian assignment عند ملاءمته لربط القياسات بالمسارات. طبّق track confirmation، والتعامل مع الفقد المؤقت، وإعادة الالتقاط، وحذف المسارات القديمة. لا تفترض حركة مستقيمة طويلة للأهداف الجوية.

```text
Detect → Track → Estimate → Predict briefly → Command UAV
      → Re-observe → Correct estimate → Repeat
```

## Ego-Motion Compensation

حركة البكسلات لا تعني تلقائيًا أن الهدف تحرك، لأن الكاميرا مثبتة على UAV متحرك. استخدم attitude وangular velocity ومعلومات حركة المركبة من PX4 لتعويض الحركة الظاهرية، ويمكن استخدام Optical Flow وتحليل حركة الخلفية بعد ذلك.

يمكن استخدام Homography كجزء من الحل، لكن لا تدّعِ أنها تثبت مشهدًا ثلاثي الأبعاد اعتباطيًا بصورة مثالية؛ اختلاف العمق والترجمة يسببان parallax. قِس خطأ التعويض عمليًا.

## المحاكاة الحرارية

- أضف Thermal Camera فعلية في SDF الخاص بـX500 variant مملوك للمشروع.
- اضبط درجات حرارة الأهداف والخلفية لإنتاج حالات thermal contrast مختلفة.
- اختبر تغيّر حرارة الهدف والخلفية والحجب الجزئي وتعدد الأهداف.
- لا تفترض أن كل بالون أو UAV حقيقي سيظهر أكثر حرارة من الخلفية.
- تعامل مع التباين الحراري كمتغير تجريبي في المحاكاة وكقيد واقعي للاختبار لاحقًا.

الحزمة `libgz-sensors8-thermal-camera` مثبتة، لكن X500 الحالي لا يحتوي كاميرا حرارية حتى الآن.

## اختيار الهدف والتوجيه

يمكن للمشغّل اختيار Target ID، أو استخدام قواعد حتمية قابلة للتفسير مثل الأكبر، أو الأسرع، أو الأطول استمرارًا، أو الأقرب إذا توفرت مسافة موثوقة، أو أولوية يحددها المشغّل. لا تستخدم semantic threat classification دون طريقة موثوقة مقاسة.

OpenCV لا يتحكم بالمحركات. PX4 مسؤول عن attitude stabilization والموتورات والتحكم منخفض المستوى وسلامة الطيران. نظام الرؤية يرسل velocity وyaw وtrajectory setpoints عالية المستوى عبر ROS 2 / PX4 Offboard.

هدف الاعتراض هو الوصول إلى Safe Standoff Point والمحافظة عليه مع إبقاء visual lock. الاصطدام المادي ليس هدفًا مطلوبًا.

لا تتطلب ملاحقة الهدف إحداثيات GPS الخاصة به، لكن لا يوصف النظام بأنه GPS-denied كامل دون localization / state estimation منفصل. IMU ورؤية الهدف وحدهما لا يحلان global localization.

## الحالة الحالية

### Simulator Phase 1: COMPLETE سابقًا

نجحت السلسلة التالية في جلسة 2026-09-19:

```text
Windows 11 → WSL2 → Ubuntu 24.04.2 → PX4 SITL
→ Gazebo Harmonic 8.15.0 → outdoor_urban → X500
→ sensors → QGroundControl → Ready → Takeoff / Hover
```

فحص البيئة بتاريخ 2026-09-20 أكد:

- Windows 11 Pro، Intel i5-12400F، RAM نحو 16 GB، وRTX 5060 8 GB.
- PX4 SITL مبني في `/home/slo/autonomous-balloon-interception-drone/PX4-Autopilot`.
- نسخة البناء `v1.18.0-beta1-694-gfd1ddbc22e`، commit `fd1ddbc22e31e7dd9718ecf686b67a065f586374`.
- Gazebo Harmonic و`gz-sim 8.15.0` مثبتان.
- QGroundControl `5.1.4` مثبت على Windows.
- العالم في `/home/slo/autonomous-balloon-interception-drone/Autonomous-Mobile-Robotics/src/robot_description/worlds/outdoor_urban.sdf`.
- مستودع البيئة عند commit `6568203187208a77a041f53f0f4577e463b410df` مع تعديل AirPressure محلي متعمد وملفات backup؛ لا تحذفها.
- لا توجد جلسة PX4 أو Gazebo نشطة وقت الفحص، لذلك Takeoff/Hover موثق من الجلسة السابقة ولم يُعَد اختباره في 2026-09-20.

### غير منجز

- Thermal Camera وthermal image stream.
- ROS 2 Jazzy؛ لم يظهر `/opt/ros` أو أمر `ros2`.
- uXRCE-DDS Agent و`px4_msgs` واتصال PX4–ROS 2.
- OpenCV وSciPy داخل Python في Ubuntu.
- عقدة الرؤية الحرارية والتتبع والتوجيه.

لا تعِد تثبيت PX4 أو Gazebo أو QGroundControl. ثبّت المكونات الناقصة فقط عند الوصول إلى مرحلتها.

## تقسيم عمل الفريق

يمكن لعضو واحد تغطية أكثر من مسار، لكن يجب تحديد مسؤول واضح لكل مخرج:

| المسار | المسؤولية | المخرج المطلوب |
| --- | --- | --- |
| المحاكاة ونموذج المركبة | X500 thermal variant، خصائص الحساس، الأهداف الحرارية، وسيناريوهات Gazebo | نموذج SDF قابل لإعادة التشغيل وthermal stream موثق |
| تكامل PX4 وROS 2 | uXRCE-DDS، `px4_msgs`، مزامنة البيانات، وتحويل الإطارات | اتصال موثق ورسائل PX4 المطلوبة ظاهرة في ROS 2 |
| الرؤية والتتبع | segmentation، morphology، المرشحات، Kalman، وربط القياسات | Tracks ثابتة بهويات مستمرة ومقاييس أداء |
| التوجيه والسلامة | اختيار الهدف، pursuit controller، Offboard، standoff، وفقد الهدف | ملاحقة مستقرة مع حدود سرعة ومسافة وسلوك آمن عند الفقد |
| الاختبار والتوثيق | السيناريوهات، تسجيل النتائج، المقارنة، وإعادة إنتاج التجارب | جداول نتائج وسجلات تشغيل وخطوات قابلة للتكرار |

كل Pull Request يجب أن يذكر المرحلة التي يغطيها، طريقة التشغيل، دليل التحقق، وأي قيود معروفة. لا تُدمج مرحلة تعتمد على مخرج سابق قبل إثبات أن المخرج السابق يعمل.

## عقود الواجهات الأولية

الأسماء التالية مقترحة للتنظيم وليست حقائق عن النظام الحالي. ثبّت الاسم النهائي ونوع الرسالة ومعدل النشر بعد مشاهدة البيانات الفعلية في المرحلتين 3–5.

| الواجهة | المصدر → المستهلك | العقد المطلوب توثيقه |
| --- | --- | --- |
| الصورة الحرارية | Gazebo → ROS 2 perception | topic الفعلي، `sensor_msgs/Image` أو النوع المستخدم، encoding، الدقة، FPS، timestamp، وcamera frame |
| معلومات الكاميرا | Gazebo/bridge → perception | intrinsics، distortion، وموضع الكاميرا بالنسبة إلى جسم المركبة |
| حركة المركبة | PX4 → tracking/guidance | attitude، angular velocity، local position/velocity، timestamps، وحالة المركبة |
| المسارات | tracking → selection/guidance | Track ID، center، velocity، prediction، confidence، last seen، وحالة المسار |
| اختيار الهدف | operator/rule → guidance | Target ID المختار، مصدر الاختيار، وقت الاختيار، وإلغاء الاختيار |
| أوامر الملاحقة | guidance → PX4 Offboard | control mode، velocity/yaw/trajectory setpoint، معدل الإرسال، الحدود، وشروط الإيقاف |

قواعد الدمج الأساسية:

- وثّق التحويل بين PX4 NED وإطارات ROS 2 المعتادة؛ لا تخلط المحاور أو إشارات yaw.
- استخدم timestamps مشتركة أو عوّض فرق الزمن، وقِس عمر الصورة قبل إرسال الأمر.
- عرّف frame names وunits في ملف واحد، وأضف اختبارًا بسيطًا لاتجاه كل محور.
- لا يدخل Offboard إلا عند توفر stream مستمر من setpoints وحالة مركبة صالحة، ويخرج منه بأمان عند فقد الهدف أو انقطاع البيانات.

## هيكل المستودع المقترح

هذا الهيكل هدف تنظيمي ويُنشأ تدريجيًا عند بدء كل حزمة:

```text
docs/
  PROJECT_HANDOFF.md
src/
  robot_description/
    models/x500_thermal/
  thermal_perception/
  target_tracking/
  visual_guidance/
config/
launch/
tests/
  scenarios/
results/
```

ضع الإعدادات القابلة للتغيير مثل thresholds وحدود السرعة ومسافة standoff في ملفات إعداد، ولا تثبتها داخل الكود. خزّن ملخصات النتائج الصغيرة في Git، واترك التسجيلات الكبيرة وملفات الفيديو وبيانات التدريب خارج المستودع مع توثيق مكانها.

## إصلاح البارومتر المحفوظ

يحتوي عنصر `<world>` في `outdoor_urban.sdf` على:

```xml
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-air-pressure-system"
        name="gz::sim::systems::AirPressure"/>
<plugin filename="gz-sim-navsat-system"
        name="gz::sim::systems::NavSat"/>
<plugin filename="gz-sim-magnetometer-system"
        name="gz::sim::systems::Magnetometer"/>
```

قبل إضافة AirPressure كان `listener sensor_baro` يعيد `never published` وتظهر `Preflight Fail: barometer 0 missing`. بعد الإصلاح نشر بيانات صحيحة مع `error_count: 0`.

## التشغيل بعد Restart

Terminal 1 — Gazebo:

```bash
cd ~/autonomous-balloon-interception-drone/Autonomous-Mobile-Robotics
export GZ_SIM_RESOURCE_PATH=$HOME/autonomous-balloon-interception-drone/PX4-Autopilot/Tools/simulation/gz/models:$HOME/autonomous-balloon-interception-drone/Autonomous-Mobile-Robotics/src/robot_description/models:$GZ_SIM_RESOURCE_PATH
gz sim -r -v 4 ./src/robot_description/worlds/outdoor_urban.sdf
```

Terminal 2 — PX4:

```bash
cd ~/autonomous-balloon-interception-drone/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=outdoor_urban \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4
```

QGroundControl على Windows:

- Type: UDP
- Local Port: `14550`
- Server Address: `<WSL-IP>:18570`
- العنوان السابق `172.23.191.143` مثال فقط وقد يتغير.

اعرف العنوان الحالي بـ:

```bash
ip addr | grep eth0
```

استخدم IPv4 الخاص بـ`eth0` دون لاحقة الشبكة. بعد التشغيل تحقق من ظهور X500، وعدم وجود `Timed out waiting for Gazebo world`، ونشر `listener sensor_baro`، ووصول QGroundControl إلى `Ready`.

## خارطة التنفيذ ومعيار قبول كل مرحلة

| المرحلة | المهمة | الحالة | تُقبل عند |
| --- | --- | --- | --- |
| 1 | PX4 SITL + Gazebo Harmonic + X500 | **Complete سابقًا** | إقلاع وتحليق ثابت مع الحساسات وQGroundControl؛ يعاد الاختبار إذا تغيّرت البيئة |
| 2 | إضافة Thermal Camera إلى X500 variant | **Next** | تحميل النموذج دون أخطاء وظهور الحساس في شجرة Gazebo |
| 3 | التحقق من thermal stream في Gazebo | Pending | قراءة frames فعلية وتوثيق topic والدقة والصيغة وFPS |
| 4 | نشر/جسر الصورة إلى ROS 2 | Pending | وصول الصورة إلى ROS 2 مع timestamps وframe id صالحين |
| 5 | التحقق من PX4 IMU في ROS 2 | Pending | وصول attitude وangular velocity باستمرار وتوثيق الإطارات والمعدل |
| 6 | ROS 2 OpenCV thermal-processing node | Pending | اشتراك ونشر مستقران وقياس زمن المعالجة دون إسقاط غير مفسر للصور |
| 7 | adaptive segmentation واستخراج المرشحين | Pending | كشف مرشحين عبر ثلاث درجات thermal contrast على الأقل |
| 8 | geometric and temporal filtering | Pending | خفض الإنذارات الكاذبة مع توثيق أثر كل مرشح |
| 9 | Kalman tracking وTrack IDs | Pending | هوية مستقرة لهدف واحد مع فقد قصير وإعادة التقاط |
| 10 | multi-target data association | Pending | الحفاظ على هويات عدة أهداف وقياس ID switches |
| 11 | ego-motion compensation | Pending | انخفاض مقاس في خطأ حركة الخلفية مع توثيق الحالات التي يفشل فيها التعويض |
| 12 | اختيار Target ID والأولوية | Pending | اختيار وتبديل وإلغاء هدف محدد دون التباس بين المسارات |
| 13 | visual pursuit controller | Pending | تقليل خطأ تمركز الهدف في اختبار مغلق الحلقة |
| 14 | PX4 velocity/yaw/trajectory setpoints | Pending | دخول Offboard وخروج آمن منه مع حدود أوامر موثقة |
| 15 | standoff behavior | Pending | المحافظة على المسافة المحددة ضمن سماحية مقاسة |
| 16 | target-loss recovery and reacquisition | Pending | توقف/سلوك آمن عند الفقد وإعادة التقاط ضمن زمن مقاس |
| 17 | test scenarios | Pending | تشغيل مصفوفة السيناريوهات مع حفظ الإعدادات والنتائج |
| 18 | benchmark وتوثيق النظام | Pending | تقرير قابل لإعادة الإنتاج يغطي الدقة والتتبع والزمن والموارد والقيود |

سيناريوهات الاختبار: هدف جوي ثابت ومتحرك ومناور؛ هدف أرضي ثابت ومتحرك؛ عدة أهداف؛ حجب جزئي؛ درجات حرارة مختلفة؛ وخلفية متغيرة الحرارة.

## أول إجراء تالٍ

أنشئ X500 variant خاصًا بالمشروع مزودًا بكاميرا حرارية، دون تعديل نموذج PX4 الأساسي مباشرة قدر الإمكان. شغّله في `outdoor_urban` وتحقق من thermal image topic وقراءة frame فعلية قبل تثبيت ROS 2 أو كتابة خوارزمية الكشف. وثّق ملف SDF واسم topic وأبعاد الصورة والصيغة وFPS.

## مكونات مفتوحة المصدر مفضلة

- Gazebo / gz-sensors thermal camera.
- ROS 2 `vision_opencv` / `cv_bridge`.
- OpenCV thresholding، morphology، contours، connected components، optical flow، وKalmanFilter.
- SciPy `linear_sum_assignment` عند استخدام Python.
- PX4 ROS 2، `px4_msgs`، uXRCE-DDS، وأمثلة PX4 Offboard.

لا تعِد تنفيذ الخوارزميات القياسية من الصفر. مساهمة المشروع هي دمج وتصميم سلسلة Thermal + IMU → Detection → Multi-Target Tracking → Motion Estimation → Guidance → PX4 Pursuit.

## مقاييس التقييم

- FPS وend-to-end latency.
- CPU وRAM usage.
- detection rate وfalse-positive rate.
- tracking success rate وtrack-loss frequency وreacquisition time.
- target-centering error.
- pursuit success rate وstandoff-distance stability.
- الأداء مقابل سرعة الهدف ومناورته وthermal contrast وعدد الأهداف.

لا تدّعِ latency أقل من 1 ms، أو تعريفًا مثاليًا للأهداف، أو تمييزًا مضمونًا بين الدرون والطيور، أو تعويضًا مثاليًا للحركة، أو GPS-denied autonomy كاملة، أو ظهورًا حراريًا مضمونًا، أو تفوقًا على AI دون تجارب.

## سجل YOLO المتوقف

أُنجزت سابقًا تجربة `YOLO26s` على Det-Fly داخل `C:\drone-vision-training`. النتائج المحفوظة تشمل:

- `best.pt` في `C:\drone-vision-training\runs\detect\detfly_yolo26s_full\weights\best.pt`.
- Test split: 1,327 صورة و1,326 target instance.
- Precision `0.980`، Recall `0.962`، mAP50 `0.981`، وmAP50-95 `0.687`.
- اختبارات فيديو خارجية أثبتت إمكانية الكشف، مع تفاوت واضح في الاستمرارية بسبب صغر الهدف واختلاف المجال.
- ملفات البيانات ونتائج التدريب وcheckpoints ومخرجات الفيديو يجب الحفاظ عليها محليًا.

هذه النتائج سجل تاريخي فقط. لا يُدمج النموذج في Gazebo ولا يُستأنف التدريب في الاتجاه الحالي إلا بطلب صريح جديد.
