<?php
include "../private/addresses.php";
include "../private/header.php";

$level = (int)file_get_contents("../private/who_am_i_level");
?>
<body>
<div id="particles-js">
    <div class="overlay">
<?php switch ($level){
    case 0:
        ?>
        <div>
            <img src="a138357a-abca-458c-bc99-1619a7c79338.svg"  alt="niveau 1" style="max-height: 500px"/>
        </div>
<?php
        break;
    case 1:?>

<?php

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