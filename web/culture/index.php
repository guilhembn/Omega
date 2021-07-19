<?php
include "../private/addresses.php";
include "../private/header.php";

$level = (int)file_get_contents("../private/who_am_i_level");
?>
<body>
<div id="particles-js">
    <div class="overlay">
        <?php if ($level == 11){?>
        On n'entend bien qu'avec les yeux.<br/>
        L'essentiel est inaudible pour les doigts.<br/>
            <small>- <a href="b612.wav">Antoine de Saint-Exupéry (probablement)</a></small>
        <?php }else{?>
        Alors on essaie de tricher ?<br/>
        <a href="../whoami">Essayez de finir celle-là d'abord...</a>
        <?php }?>
    </div>
</div>

<script src="../lib/particles.min.js"></script>
<script src="../static/particleapp.js"></script>
<script src="../static/incremental.js"></script>
</body>