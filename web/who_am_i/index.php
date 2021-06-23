<?php
include "../private/addresses.php";
include "../private/header.php";

$level = (int)file_get_contents("../private/who_am_i_level");
?>
<body>
<div id="particles-js">
    <div class="overlay whoami">
<?php switch ($level){
    case 0:
        ?>
        <div>
            <img src="a138357a-abca-458c-bc99-1619a7c79338.svg"  alt="niveau 1" />
        </div>
<?php
        break;
    case 1:?>
        <div style="display: inline">
            <img src="8a1213c6-a8e8-466f-af22-d9a8f5d61369.png" alt="niveau 2" style="max-width: 33.33%; display: inline; vertical-align: middle; margin: 0;">
            <div style="max-width: 33.33%; display: inline; vertical-align: middle; margin: 0;">A</div>
            <img src="67d19ae8-d413-11eb-b8bc-0242ac130003.png" alt="niveau 2" style="max-width: 33.33%; display: inline; vertical-align: middle; margin: 0;">
        </div>
<?php
        break;
    case 2:?>
        <div>
            <img src="41ced434-d416-11eb-b8bc-0242ac130003.png" alt="niveau 3" />
        </div>
<?php
        break;
    case 3:?>
        <div>
            <img src="31813e6c-d418-11eb-b8bc-0242ac130003.png" alt="niveau 4" />
        </div>
        <?php
        break;
    case 4:?>
        <div style="margin: 50px;">
            C7dPqrmDWxs
        </div>
        <?php
        break;

}
?>

        <div>
            <button onclick="check()">C'est moi !<span class="btn__glitch">C'est moi !</span>
        </div>
    </div>
</div>

<script src="../lib/particles.min.js"></script>
<script src="../static/particleapp.js"></script>
<script src="../static/incremental.js"></script>
<script src="../static/whoami.js"></script>
</body>