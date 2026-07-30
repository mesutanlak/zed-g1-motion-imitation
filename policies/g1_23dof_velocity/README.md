# G1 23-DOF denge politikası

Bu klasördeki `policy.onnx`, bu bilgisayarda resmi
`unitreerobotics/unitree_rl_mjlab` içindeki `Unitree-G1-23Dof-Flat` göreviyle
tamamlanan 10.000 iterasyonluk eğitimin dışa aktarılmış çıktısıdır.

Kaynak checkpoint:

`/home/misafir/unitree_rl_mjlab/logs/rsl_rl/g1_23dof_velocity/2026-07-23_09-53-18_resume_official_23dof/model_10000.pt`

Kaynak dağıtım çıktısı:

`/home/misafir/unitree_rl_mjlab/deploy/robots/g1_23dof/config/policy/velocity/v0/`

Politika sözleşmesi:

- giriş: `obs`, `[1, 80]`
- çıkış: `actions`, `[1, 23]`
- kontrol aralığı: 0,02 s (50 Hz)
- komut: sıfır doğrusal/açısal hız; yerinde dengeli duruş
- fiziksel robot çıkışı: kapalı

`deploy.yaml` eklem sırasını, varsayılan açıları, eylem ölçeklerini ve resmi
dağıtım kazançlarını taşır. Isaac canlı köprüsü alt gövde ve bel hedeflerini bu
politikadan, üst gövde hedeflerini ise güvenlik süzgecinden geçmiş GMR
referansından alır.
