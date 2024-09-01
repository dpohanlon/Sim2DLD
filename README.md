<p align="center">
  <img width="400" height="400" src="https://github.com/dpohanlon/Sim2DLD/blob/main/assets/lidar.png">
  <br>
  Two dimensional LIDAR simulation.
</p>

Installation
---
Install from PyPI
```bash
pip install brioche-enrichment
```

or install from the Github repository
```bash
git clone git@github.com:dpohanlon/brioche.git
pip install -r requirements.txt .
```

Usage
---
Prepare some data in a contingency table format, with row and column set annotations
```python
row_names = ["a", "b", "c", ...]
col_names = ["l", "m", "n", ...]

data = np.array([[30, 27, 10, ...], [28, 25, 11, ...], [31, 29, 15, ...], ...])
```
