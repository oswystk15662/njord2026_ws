


function gplsx_isInteger(x) {
    return (typeof x === 'number') && (x % 1 === 0);
}

function gpls_woo_rfq_plus_is_Decimal(evt) {
    evt = (evt) ? evt : window.event;
    var charCode = (evt.which) ? evt.which : evt.keyCode;
    console.log(charCode);
    if ((charCode >= 48 && charCode <= 57) || ((charCode >= 96 && charCode <= 105))
        || charCode==8 || charCode==46 || charCode==110 || (charCode >= 37 && charCode <= 40)) {
        return true;}
    return false;

}

function gpls_woo_rfq_plus_is_validate_required(evt) {
    console.log(event.target.value);
    if (!event.target.value) {
        jQuery('.wc-block-components-checkout-place-order-button').prop("disabled", true);
        jQuery(".gpls_woo_rfq_plus_customer_bid_text").show();

    }else{
        jQuery('.wc-block-components-checkout-place-order-button').removeAttr("disabled");
        jQuery(".gpls_woo_rfq_plus_customer_bid_text").hide();
    }
    return true;
}

function gpls_woo_rfq_plus_is_Integer(evt) {
    var charCode = (evt.which) ? evt.which : event.keyCode
    if (charCode > 31 && (charCode < 48 || charCode > 57))
        return false;
    return true;
}


function rfqtk_main(){
    jQuery('.update_rfq_cart').on('click', function () {

        var form = jQuery(".nf-form-layout").remove();

        if ( jQuery( '#rfq_phone' ).length && jQuery( '#rfq_phone' ).is(':visible')) {
            jQuery('#rfq_phone').removeClass("required");
            jQuery('#rfq_phone').removeClass("required");
            jQuery('#rfq_phone').attr("required", false);
        }

        if ( jQuery( '#rfq_company' ).length && jQuery( '#rfq_company' ).is(':visible')) {
            jQuery('#rfq_company').removeClass("required");
            jQuery('#rfq_company').attr("required", false);
        }


        if ( jQuery( '#rfq_billing_country' ).length && jQuery( '#rfq_billing_country' ).is(':visible')) {
            jQuery('#rfq_billing_country').removeClass("required");
            jQuery('#rfq_billing_country').attr("required", false);
        }

        if ( jQuery( '#rfq_state_select' ).length && jQuery( '#rfq_state_select' ).is(':visible')) {
            jQuery('#rfq_state_select').removeClass("required");
            jQuery('#rfq_state_select').attr("required", false);
        }

        if ( jQuery( '#rfq_address' ).length && jQuery( '#rfq_address' ).is(':visible')) {
            jQuery('#rfq_address').removeClass("required");
            jQuery('#rfq_address').attr("required", false);
        }

        if ( jQuery( '#rfq_city' ).length && jQuery( '#rfq_city' ).is(':visible')) {
            jQuery('#rfq_city').removeClass("required");
            jQuery('#rfq_city').attr("required", false);
        }

        if ( jQuery( '#rfq_zip' ).length && jQuery( '#rfq_zip' ).is(':visible')) {
            jQuery('#rfq_zip').removeClass("required");
            jQuery('#rfq_zip').attr("required", false);
        }

        if ( jQuery( '#rfq_message' ).length && jQuery( '#rfq_message' ).is(':visible')) {
            jQuery('#rfq_message').removeClass("required");
            jQuery('#rfq_message').attr("required", false);
        }




    });

    if ( jQuery( '#gpls_woo_rfq_customer_note' ).length ) {
        //  jQuery('#gpls_woo_rfq_customer_note').attr("required", true);
    }





}


jQuery(window).on("load",function () {
    rfqtk_main();




});

jQuery( document ).ajaxComplete(function() {
    rfqtk_main();


});


function hideit(){
    jQuery( 'form.cart .button.single_add_to_cart_button' ).hide();
    jQuery( '.single-product div.product p.price' ).hide();
    jQuery( 'form.cart .button.gpls_rfq_set' ).show();
}
function showit(){
    jQuery( 'form.cart .button.single_add_to_cart_button' ).hide();
    jQuery( '.single-product div.product p.price' ).hide();
    jQuery( 'form.cart .button.gpls_rfq_set' ).show();
}

