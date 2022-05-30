<head>
    <title>Guerrero compte à rebours</title>
    <meta charset="UTF-8" >
    <meta lang="fr">
    <link href="style.css" rel="stylesheet">
</head>
<body>
<p id="cd"></p>

<script>
// Set the date we're counting down to
var countDownDate = <?php
$date = new DateTime('07/12/2021 11:04:00'); // format: MM/DD/YYYY
echo $date->format('U') * 1000; ?>

// Update the count down every 1 second
var x = setInterval(function() {

    // Get today's date and time
    var now = new Date().getTime();

    // Find the distance between now and the count down date
    var distance = countDownDate - now;

    // Time calculations for days, hours, minutes and seconds
    var left = distance;
    var days = Math.floor(left / 86400000);
    left -= days * 86400000;
    var hours = Math.floor(left / 3600000);
    left -= hours * 3600000;
    var minutes = Math.floor(left / 60000);
    left -= minutes * 60000;
    var seconds = Math.floor(left / 1000);

    // Display the result in the element with id="demo"
    document.getElementById("cd").innerHTML = days + "j " + hours + "h "
        + minutes + "m " + seconds + "s ";

    // If the count down is finished, write some text
    if (distance < 0) {
        clearInterval(x);
        document.getElementById("cd").innerHTML = "C'EST PARTI !";
    }
}, 1000);
</script>
</body>