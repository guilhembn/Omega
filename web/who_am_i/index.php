<?php
include "../private/addresses.php";
include "../private/header.php";

$level = (int)file_get_contents("../private/who_am_i_level");
?>
<body>
<div id="particles-js">
    <div class="overlay">
        level: <?=$level?>
    </div>
</div>

<script src="../lib/particles.min.js"></script>
<script src="../static/particleapp.js"></script>
<script src="../static/incremental.js"></script>
</body>
'