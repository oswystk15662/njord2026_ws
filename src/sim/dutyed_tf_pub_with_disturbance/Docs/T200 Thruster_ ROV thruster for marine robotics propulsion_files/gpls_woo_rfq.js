


jQuery.ajaxSetup({ cache: false });

jQuery(document).ready(function() {


    try {



        jQuery( '.related .woocommerce-Price-amount >*,' +
            '.related .from >*,' +
            '.related .price >*,' +
            '.related .total >*,' +
            '.related .amount >*,' +
            '.related  .bundle_price >*').show();


        jQuery( '.related .woocommerce-Price-amount >*,' +
            '.related .from >*,' +
            '.related .price >*,' +
            '.related .total >*,' +
            '.related .amount >*,' +
            '.related  .bundle_price >*').attr('style','visibility: visible !important;');






//note
     //   jQuery( '.single-product div.product form.cart' ).attr('style','visibility: visible !important');



 ////note

        jQuery( '.qodef--single' ).attr('style','visibility: visible !important');



        jQuery('.required').attr("required", true);

        jQuery('form.checkout').removeAttr('novalidate');
        jQuery('.required').attr("required", true);


        jQuery('.gpls-woo-rfq_update-rfq-cart_button').on('click', function () {

            jQuery('input[name="gpls-woo-rfq_update"]').val(true);
            jQuery('input[name="gpls-woo-rfq_checkout"]').val(false);

            //jQuery("#rfqform").rules('remove');

            jQuery('#rfq_fname').attr("required", false);
            jQuery('#rfq_lname').attr("required", false);
            jQuery('#rfq_email_customer').attr("required", false);


            jQuery('#rfq_fname').removeClass("required");
            jQuery('#rfq_lname').removeClass("required");
            jQuery('#rfq_email_customer').removeClass("required");


        });


        jQuery('.gpls-woo-rfq_checkout_button').on('click', function () {

            jQuery('.required').attr("required", true);

            jQuery('#rfq_fname').attr("required", true);
            jQuery('#rfq_lname').attr("required", true);
            jQuery('#rfq_email_customer').attr("required", true);

            jQuery('input[name="gpls-woo-rfq_update"]').val(false);
            jQuery('input[name="gpls-woo-rfq_checkout"]').val(true);

        });

        jQuery('.rfq_billing_country').on('change', function () {


            if (jQuery('#rfq_billing_country').val() !== 'US') {
                jQuery('#rfq_state_select').val("");
                jQuery('#rfq_state_select').hide();
                jQuery('#rfq_state_text').show();
                jQuery('#rfq_state_select').attr("required", false);
                jQuery('#rfq_state_text').attr("required", true);
            } else {
                jQuery('#rfq_state_text').val("");
                jQuery('#rfq_state_text').hide();
                jQuery('#rfq_state_select').show();
                jQuery('#rfq_state_select').attr("required", true);
                jQuery('#rfq_state_text').attr("required", false);
            }

        });


        jQuery('.gpls_rfq_set').on('click', function () {
            var variation_id = jQuery("[name='variation_id']").val();
            var rfq_product_id = jQuery("[name='add-to-cart']").val();

            jQuery("[name='rfq_product_id']").val(1);
            jQuery("[name='rfq_id']").val(rfq_product_id);
        });

        jQuery(".product-quantity").on('change', function () {


            jQuery('.shop_table.cart').closest('form').find('input[name="update_cart"]').prop('disabled', false);

        });


        jQuery( '.related .woocommerce-Price-amount >*,' +
            '.related .from >*,' +
            '.related .price >*,' +
            '.related .total >*,' +
            '.related .amount >*,' +
            '.related  .bundle_price >*').show();


        jQuery( '.related .woocommerce-Price-amount >*,' +
            '.related .from >*,' +
            '.related .price >*,' +
            '.related .total >*,' +
            '.related .amount >*,' +
            '.related  .bundle_price >*').attr('style','visibility: visible !important;');




    }catch(err){}

} );

jQuery( document ).ready( function() {



   try {

       // jQuery( '.single_add_to_cart_button' ).show();

       if (jQuery('#rfq_billing_country').val() !== 'US') {
           jQuery('#rfq_state_select').val("");
           jQuery('#rfq_state_select').hide();
           jQuery('#rfq_state_text').show();
           jQuery('#rfq_state_select').attr("required", false);
           jQuery('#rfq_state_text').attr("required", true);
       } else {
           jQuery('#rfq_state_text').val("");
           jQuery('#rfq_state_text').hide();
           jQuery('#rfq_state_select').show();
           jQuery('#rfq_state_select').attr("required", true);
           jQuery('#rfq_state_text').attr("required", false);
       }

       jQuery('form.checkout').on('click', function () {
           jQuery('form.checkout').removeAttr('novalidate');
       });
   }catch(err){}

});

jQuery(".remove_session_files a").click(function (e) {
    try {
        e.preventDefault();
    }catch(err){}
} );







