# 좌표계 변환 (LIGO / gnss_comm 기준)

이 문서는 저장소에서 사용하는 **WGS84 ECEF**, **LLA(위경도)**, **ENU** 간 관계와 LIGO 맵 메타데이터 식을 정리한다. 구현의 기준은 `gnss_comm`의 `gnss_utility.cpp`와 LIGO의 `laserMapping.cpp`, `Indoor_Processing.cpp`, `Indoor_GridMapRegistry.cpp` 등이다.

---

## 기호·상수


| 기호                                                  | 의미                                                                    |
| --------------------------------------------------- | --------------------------------------------------------------------- |
| $\mathbf{p}_{\mathrm{ecef}} = [X,Y,Z]^{\mathsf{T}}$ | WGS84 ECEF (m)                                                        |
| $(\varphi,\lambda,h)$                               | 위도·경도(deg)·타원체 높이 (m)。`gnss_comm`에서는 `lla << lat_deg, lon_deg, alt_m` |
| $R_{\mathrm{ecef}\leftarrow\mathrm{enu}}$           | ENU 단위 벡터를 ECEF로 옮기는 $3\times3$ 회전 (`R_ecef_enu`)                     |
| $D2R=\pi/180$, $R2D=180/\pi$                        | 도 ↔ 라디안                                                               |


WGS84 (`gnss_constant.hpp`):

- $a = 6378137$ m (장반경)
- $e^2 = 6.69437999014\times 10^{-3}$ (첫 번째 편심률 제곱)

---

## 1. LLA → ECEF (`geo2ecef`)

$$
N = \frac{a}{\sqrt{1 - e^2 \sin^2\varphi}}
$$

$$
\begin{aligned}
X &= (N + h)\cos\varphi\cos\lambda
\end{aligned}
$$

$$
\begin{aligned}
Y &= (N + h)\cos\varphi\sin\lambda 
\end{aligned}
$$

$$
\begin{aligned}
Z &= \bigl(N(1-e^2) + h\bigr)\sin\varphi
\end{aligned}
$$

$\varphi,\lambda$는 구현에서 라디안으로 변환해 사용한다 (`lla(0)*D2R`, `lla(1)*D2R`).

---

## 2. ECEF → LLA (`ecef2geo`)

닫힌 단일식이 아니라 RTKLIB 계열의 절차로 위도·경도·고도를 구한다. 요지는 다음과 같다.

- $p = \sqrt{X^2+Y^2}$, $b^2=a^2(1-e^2)$, $e'^2=(a^2-b^2)/b^2$
- 보조 기하로 $\theta$에 대한 $\sin\theta,\cos\theta$ 계산 후
- $\tan\varphi = \dfrac{Z + e'^2 b \sin^3\theta}{p - a e^2 \cos^3\theta}$, $\lambda = \operatorname{atan2}(Y,X)$
- 고도 $h = p/\cos\varphi - N$ (해당 $\varphi$에서의 수직반경 $N$)

출력: 위도·경도는 **deg**, 높이는 **m**. 세부는 `gnss_comm` 소스를 따른다.

---

## 3. ECEF ↔ ENU 회전 (`geo2rotation`, `ecef2rotation`)

### 3.1 `geo2rotation(ref_geo)` — ENU → ECEF

참조점의 위경도 $\varphi,\lambda$ (rad):

$$
R_{\mathrm{ecef}\leftarrow\mathrm{enu}} =
\begin{bmatrix}
-\sin\lambda & -\sin\varphi\cos\lambda & \cos\varphi\cos\lambda & \\
\cos\lambda & -\sin\varphi\sin\lambda & \cos\varphi\sin\lambda & \\
0 & \cos\varphi & \sin\varphi
\end{bmatrix}
$$

### 3.2 `ecef2rotation(ref_ecef)`

$$
R_{\mathrm{ecef}\leftarrow\mathrm{enu}} = \texttt{geo2rotation}\bigl(\texttt{ecef2geo}(\mathbf{p}_{\mathrm{ref,ecef}})\bigr)
$$

LIGO에서 그리드 YAML을 저장할 때 `ecef2rotation(first_gps_ecef)`로 넣는 $R$과 같은 정의다.

### 3.3 변위 (작은 벡터)

$$
\Delta\mathbf{p}_{\mathrm{ecef}} = R*{\mathrm{ecef}\leftarrow\mathrm{enu}}\Delta\mathbf{p}_{\mathrm{enu}}, \qquad
\Delta\mathbf{p}_{\mathrm{enu}} = R_{\mathrm{ecef}\leftarrow\mathrm{enu}}^{\mathsf{T}}\Delta\mathbf{p}_{\mathrm{ecef}}
$$

($R$이 직교행렬이면 역행렬 = 전치.)

---

## 4. ECEF 벡터 → ENU (`ecef2enu`)

참조 LLA $\mathbf{r}_{\mathrm{lla}}$와 ECEF 벡터 $\mathbf{v}_{\mathrm{ecef}}$에 대해:

$$
R_{\mathrm{enu}\leftarrow\mathrm{ecef}} =
\begin{bmatrix}
-\sin\lambda & \cos\lambda & 0 \\
-\sin\varphi\cos\lambda & -\sin\varphi\sin\lambda & \cos\varphi \\
\cos\varphi\cos\lambda & \cos\varphi\sin\lambda & \sin\varphi
\end{bmatrix}
$$

$$
\mathbf{v}_{\mathrm{enu}} = R_{\mathrm{enu}\leftarrow\mathrm{ecef}}\mathbf{v}_{\mathrm{ecef}}
$$

구현상 $R_{\mathrm{enu}\leftarrow\mathrm{ecef}} = R_{\mathrm{ecef}\leftarrow\mathrm{enu}}^{\mathsf{T}}$ 와 일치한다.

---

## 5. LIGO 맵 메타데이터 (`ecef_from_enu`)

YAML에 명시된 식:

**ENU → ECEF (위치)**

$$
\mathbf{p}_{\mathrm{ecef}} = \mathbf{a}_{\mathrm{ecef}} + R_{\mathrm{ecef}\leftarrow\mathrm{enu}}\mathbf{p}_{\mathrm{enu}}
$$

- $\mathbf{a}_{\mathrm{ecef}}$: `anchor_ecef_m`
- $\mathbf{p}_{\mathrm{enu}}$: 맵/스캔 ENU (m)
- $R_{\mathrm{ecef}\leftarrow\mathrm{enu}}$: `R_ecef_enu`

**ECEF → 맵 ENU**

$$
\mathbf{p}_{\mathrm{enu}} = R_{\mathrm{ecef}\leftarrow\mathrm{enu}}^{\mathsf{T}}\bigl(\mathbf{p}_{\mathrm{ecef}} - \mathbf{a}_{\mathrm{ecef}}\bigr)
$$

`Indoor_GridMapRegistry`에서 `frame_id: enu`일 때 2D 그리드 투영에 사용한다.

---

## 6. GNSS 앵커 기준 ENU ↔ ECEF (융합 위치 등)

$\mathbf{a}_{\mathrm{ecef}}=\texttt{geo2ecef}(\mathbf{lla}_{\mathrm{anc}})$,  
$R_{\mathrm{ecef}\leftarrow\mathrm{enu}}=\texttt{geo2rotation}(\mathbf{lla}_{\mathrm{anc}})$ 일 때:

$$
\mathbf{p}_{\mathrm{ecef}} = \mathbf{a}_{\mathrm{ecef}} + R_{\mathrm{ecef}\leftarrow\mathrm{enu}}\mathbf{p}_{\mathrm{enu}}
$$

$$
\mathbf{p}_{\mathrm{enu}} = R_{\mathrm{ecef}\leftarrow\mathrm{enu}}^{\mathsf{T}}\bigl(\mathbf{p}_{\mathrm{ecef}} - \mathbf{a}_{\mathrm{ecef}}\bigr)
$$

`parameters.cpp`의 ENU↔ECEF 유틸과 동일한 구조다.

---

## 7. 시스템 ENU 앵커 vs 맵 ENU 앵커 (`Indoor_Processing`)

맵 $(\mathbf{a}_{\mathrm{map}}, R_{\mathrm{map}})$, 시스템 $(\mathbf{a}_{\mathrm{sys}}, R_{\mathrm{sys}})$:

$$
R_{\mathrm{sys}\to\mathrm{map}} = R_{\mathrm{map}}^{\mathsf{T}} R_{\mathrm{sys}}
$$

$$
\mathbf{t}_{\mathrm{sys}\to\mathrm{map}} = R_{\mathrm{map}}^{\mathsf{T}}(\mathbf{a}_{\mathrm{sys}} - \mathbf{a}_{\mathrm{map}})
$$

---

## 요약 표


| 변환               | 식                                                                                                                        |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------ |
| LLA→ECEF         | $X=(N+h)\cos\varphi\cos\lambda$ 등 (`geo2ecef`)                                                                           |
| ECEF→LLA         | `ecef2geo`                                                                                                               |
| ENU 변위→ECEF      | $\Delta\mathbf{p}_{\mathrm{ecef}} = R*{\mathrm{ecef}\leftarrow\mathrm{enu}}\Delta\mathbf{p}_{\mathrm{enu}}$              |
| ECEF 변위→ENU      | $\Delta\mathbf{p}_{\mathrm{enu}} = R*{\mathrm{ecef}\leftarrow\mathrm{enu}}^{\mathsf{T}}\Delta\mathbf{p}_{\mathrm{ecef}}$ |
| LIGO 위치 ENU↔ECEF | $\mathbf{p}_{\mathrm{ecef}} = \mathbf{a}_{\mathrm{ecef}} + R\mathbf{p}_{\mathrm{enu}}$, 역은 $R^{\mathsf{T}}$             |


---

## 참고

- Python 스크립트 `scripts/pcd_enu_to_ecef.py`는 위와 동일한 `geo2ecef` / `ecef2geo` / `geo2rotation` / `ecef2rotation`을 이식해 두었다.
- ECEF만으로 위경도가 필요하면: $\texttt{ecef2geo}(\mathbf{p}_{\mathrm{ecef}})$ → $(\varphi,\lambda,h)$.

