jQuery(function($){ // use jQuery code inside this to avoid "$ is not defined" error
    $('.blog_loadmore').click(function(){

        var button = $(this),
            data = {
            'action': 'loadmore',
            'query': blog_loadmore_params.posts, // that's how we get params from wp_localize_script() function
            'page' : blog_loadmore_params.current_page,
            'get_category' : $(this).data('category')
        };

        $.ajax({ // you can also use $.post here
            url : blog_loadmore_params.ajaxurl, // AJAX handler
            data : data,
            type : 'POST',
            beforeSend : function ( xhr ) {
                button.text('Loading...'); // change the button text, you can also add a preloader image
            },
            success : function( data ){
                if( data ) {
                    button.text( 'Keep Exploring' );
                    $('#main').find('article:last-of-type').after( data ); // insert new posts
                    blog_loadmore_params.current_page++;

                    if ( blog_loadmore_params.current_page == blog_loadmore_params.max_page )
                        button.text('No More Posts');


                } else {
                    button.text('No More Posts'); // if no data, remove the button as well
                }
            }
        });
    });
});
