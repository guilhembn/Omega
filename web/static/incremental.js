$( document ).ready(function() {
    const timeBetweenAppear = 2000;
    function reveal(elt){
        elt.slideToggle(1000);
    }
    $(".incremental").children().each(function(i){
        setTimeout(reveal, timeBetweenAppear * (i+1), $(this));
    })

});

