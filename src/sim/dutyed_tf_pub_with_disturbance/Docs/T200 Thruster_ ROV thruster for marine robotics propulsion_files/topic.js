jQuery( window ).load(function() {
    
    function load_topics(){
        jQuery( ".topic.topic-col" ).each(function( index ) {
        var topic = jQuery(this).data("topic");
        var image = jQuery(this).data("image");
        var content = jQuery(this).data("content");
        var str = '&topic=' + topic + '&image=' + image + '&content=' + content + '&action=discourse_topic_ajax';
        jQuery.ajax({
            type: "POST",
            dataType: "html",
            url: topic_shortcode.ajaxurl,
            data: str,
            success: function(data){
                var jQuerydata = jQuery(data);
                if(jQuerydata.length){
                    jQuery("#topic-" + topic).append(jQuerydata);
                } 
            },
            error : function(jqXHR, textStatus, errorThrown) {
                console.log(errorThrown);
            }

        });
        });
        return false;
    }

    load_topics();
});