# Path_Planning
Multi-objective path planning for Mars Rover

<br/>
-------- NOTES --------------------------------------------------------------- <br/>

- `std::priority_queue` is a "container adaptor". By default std::vector is the container used inside.


- for weak pointers (check): <br/>
```
if (auto temp = N_ptr.lock())   // if Jack there
        std::cout << "ok";
else
        std::cout << "The object is not there.";
```
