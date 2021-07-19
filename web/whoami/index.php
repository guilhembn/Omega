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
        <div>
            <img src="72b039fe-d4e2-11eb-b8bc-0242ac130003.png" alt="niveau 2" />
        </div>
<?php
        break;
    case 2:?>
        <div>
            <img src="bd2f7818-fd91-4a7f-a285-2d56f09aa5eb.png" alt="niveau 3" />
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
    case 5:
        ?>
        <div style="display: inline">
            <img src="8a1213c6-a8e8-466f-af22-d9a8f5d61369.png" alt="niveau 6" style="max-width: 33.33%; display: inline; vertical-align: middle; margin: 0;">
            <div style="max-width: 33.33%; display: inline; vertical-align: middle; margin: 0;">A</div>
            <img src="67d19ae8-d413-11eb-b8bc-0242ac130003.png" alt="niveau 6" style="max-width: 33.33%; display: inline; vertical-align: middle; margin: 0;">
        </div>
        <?php
        break;
    case 6:
        ?>
        <div style="margin: 50px;">
            Qu'il est beau, qu'il est bon [...] à petites gorgées.
        </div>
        <?php
        break;
    case 7:
        ?>
        <div style="margin: 50px;">
            Vous savez que ce sont les mâles qui portent les oeufs ?
        </div>
        <?php
        break;
    case 8:
        ?>
        <div>
            <img src="41ced434-d416-11eb-b8bc-0242ac130003.png" alt="niveau 9" />
        </div>
        <?php
        break;
    case 9:
        ?>
        <div>
            <img src="341fe23c-d534-11eb-b8bc-0242ac130003.png" alt="niveau 10" />
        </div>
        <?php
        break;
    case 10:
        ?>
        <div style="margin: 50px;">
            Up. Bottom. Charm. Top. Strange. Down.
        </div>
        <?php
        break;
    case 11:
        ?>
        <i>PR2</i><br /><i>Mot de passe :</i><br />It's me !
        <?php
        break;
}
    if ($level < 11){ ?>
        <div>
            <button onclick="check()">C'est moi !<span class="btn__glitch">C'est moi !</span>
        </div>
<?php } ?>
    </div>
</div>

<script src="../lib/particles.min.js"></script>
<script src="../static/particleapp.js"></script>
<script src="../static/incremental.js"></script>
<script src="../static/whoami.js"></script>
</body>