# Obiettivo

si vuole imporre un vincolo di velocita cartesiana all`end effector. ad esempio

$$ -v_{max} \leq v_z \leq v_{max}  $$

Durante la fase di impatto.

### cosa si e` visto 
nel sorgente di task esiste gia 

```c++
tasks::qp::BoundedSpeedConstr
```
che deriva da
```c++
ConstraintFunction<GenInequality>
```
cioè una constraint del tipo

$$ l \leq Ax \leq  u $$

Non è un limite sulle velocità dei giunti.

È un limite sulla velocità di un body

### Come funziona
La funzione principale è
``` c++
addBoundedSpeed(
    mbs,
    bodyName,
    bodyPoint,
    dof,
    lowerSpeed,
    upperSpeed);
```
dove

- bodyName = nome del link (es. "left_tool")
- bodyPoint = punto sul body (normalmente 
- Eigen::Vector3d::Zero())
- dof = matrice di selezione delle componenti della velocità
- lowerSpeed / upperSpeed = limiti.

### Come integrarla in mc_rtc
Qui entra in gioco mc_rtc.

mc_rtc non usa direttamente tasks::qp::BoundedSpeedConstr.

Fa sempre un wrapper.

Lo schema è identico a quello di CollisionsConstraint

```
mc_solver::BoundedCartesianVelocityConstraint
        |
        |
        +------ tasks::qp::BoundedSpeedConstr
```

costruttore

creare

```
tasks::qp::BoundedSpeedConstr
addToSolverImpl()
```

fare

```
boundedSpeed_.addToSolver(...);
removeFromSolverImpl()
```

fare

boundedSpeed_.removeFromSolver(...);
update()

richiamare

```
boundedSpeed_.update(...);
```

(se necessario; la gestione dipende da come Tasks aggiorna internamente il vincolo).



### Implementazione passo passo

#### step 1 Dichiarazione nel controller e import in the header file
- #include <mc_solver/BoundedSpeedConstr.h>
- std::shared_ptr<mc_solver::BoundedSpeedConstr> speedConstr_;


