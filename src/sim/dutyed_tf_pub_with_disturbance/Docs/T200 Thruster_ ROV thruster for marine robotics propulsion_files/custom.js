(function($){
    //prettyDropDown taken from https://thdoan.github.io/pretty-dropdowns/
    //$('select').prettyDropdown();
    // Get started!

    // $('li.dropdown').on('click', function() {
    //     console.log('called');
    //     var $el = $(this);
    //     if ($el.hasClass('open')) {
    //         var $a = $el.children('a.dropdown-toggle');
    //         if ($a.length && $a.attr('href')) {
    //             location.href = $a.attr('href');
    //         }
    //     }
    // });
    // $('.product-search-field').focus();

    $( ".tab-link" ).each(function() {
        $(this).click(function(e) {
            var tab = $(this).attr('data-target');
            var wpadminbar = $('#wpadminbar').height();

            if(wpadminbar){
                var navbar_height = $('.navbar.fixed-top').height() + 50 + wpadminbar;
            }else{
                var navbar_height = $('.navbar.fixed-top').height() + 50;
            }

            if($(tab).hasClass("show")){
                $('html,body').animate({
                    scrollTop: $(tab).offset().top
                }, 500);
            }else{
                $(tab).collapse('show');
                $(tab).on('shown.bs.collapse', function (e) {
                    $('html,body').animate({
                        scrollTop: $(tab).offset().top
                    }, 500);
                });
            }
        });
    });

	$("li.dropdown-submenu span").click(function(e) {
        var plusIconCOntainer = $(this);
        var icon = $(plusIconCOntainer).find("i");
        var isObjVisible = $(this).parent().next('ul.dropdown-menu').is(":visible");

        if (isObjVisible == true) {
            $(this).parent().next('ul.dropdown-menu').css("display", "none");
            $(icon).removeClass("button-minusaccordian");
            $(icon).addClass("button-accordian");
            // $(this).removeAttr("tabindex").attr("tabindex", "1").focus();
        } else {
            var isParent = $(this).closest(".dropdown-submenu .dropdown-menu");
            $(".dropdown-submenu  a>  span i").removeClass("button-minusaccordian");
            $(".dropdown-submenu  a > span i").addClass("button-accordian");
            $(".dropdown-submenu .dropdown-menu").hide();

            $(this).parent().next('ul.dropdown-menu').css("display", "block");
            $(icon).removeClass("button-accordian");
            $(icon).addClass("button-minusaccordian");
            /*	if (isParent) {
				$(isParent).show();
				$(isParent).parent().find(">span i").addClass("fa-minus");
      }*/
            //  $(this).removeAttr("tabindex").attr("tabindex", "1").focus();
        }
        e.stopPropagation();
        e.preventDefault();
    });

    //Slider

    $('.customer-logos').slick({
        slidesToShow: 4,
        slidesToScroll: 1,
        autoplay: true,
        autoplaySpeed: 1500,
        prevArrow:'<i class="fa-chevron-left"></i>',
        nextArrow:'<i class="fa-chevron-right"></i>',
        arrows: true,
        dots: false,
        pauseOnHover: false,
        // mobileFirst:true,
        responsive: [
            {
                breakpoint: 1025,
                settings: {
                    slidesToShow: 3,

                  }
            },
            {
                breakpoint: 813,
                settings: {
                    slidesToShow: 3
                }
            },
            {
                breakpoint: 640,
                settings: {
                    slidesToShow: 2,
                    centerMode: true,
                    variableWidth: true,
                    centerMode: true,
                    // centerPadding: '10px',
                    // slidesToScroll: 0
                }
        }]
    });

    // $('.slider-for').slick({
    //     slidesToShow: 4,
    //     slidesToScroll: 1,
    //     autoplay: true,
    //     autoplaySpeed: 1500,
    //     prevArrow:"<img class='a-left control-c prev slick-prev' src='http://localhost/blueroboticsDev/wp-content/uploads/2019/09/Group_2.png'>",
    //     nextArrow:"<img class='a-right control-c next slick-next' src='http://localhost/blueroboticsDev/wp-content/uploads/2019/09/Group_1.png'>",
    //     arrows: true,
    //     dots: false,
    //     pauseOnHover: false,
    //     responsive: [{
    //         breakpoint: 768,
    //         settings: {
    //             slidesToShow: 4             }
    //     }, {
    //         breakpoint: 520,
    //         settings: {
    //             slidesToShow: 2
    //         }
    //     }]
    // });

    $('.slider-for').slick({
        slidesToShow: 1,
        slidesToScroll: 1,
        arrows: false,
        fade: true,
        asNavFor: '.slider-nav',
        mobileFirst:true,//add this one
      });
      $('.slider-nav').slick({
        slidesToShow: 4,
        slidesToScroll: 1,
        asNavFor: '.slider-for',
        dots: false,
        arrows: true,
        prevArrow:"<i class='fa fa-chevron-left' aria-hidden='true'></i>",
        nextArrow:"<i class='fa fa-chevron-right' aria-hidden='true'></i>",
        centerMode: false,
        focusOnSelect: true,
        // mobileFirst:true,//add this one
        responsive: [
            {
                breakpoint: 1280,
                settings: {
                    unslick: true
                  }
            },
            {
                breakpoint: 1025,
                settings: {
                    slidesToShow: 3,
                    centerMode: true,
                    centerPadding: '0px',
                    slidesToScroll: 1
                  }
            },
            {
                breakpoint: 950,
                settings: {
                    slidesToShow: 3,
                    centerMode: true,
                    centerPadding: '0px',
                    slidesToScroll: 1
                }
            },
            {
                breakpoint: 768,
                settings: {
                    slidesToShow: 3,
                    centerMode: true,
                    centerPadding: '0px',
                    slidesToScroll: 1
                }
            },

            {
                breakpoint: 480,
                settings: {
                    slidesToShow: 3,
                    centerMode: true,
                    centerPadding: '10px',
                    slidesToScroll: 1
                }
            }]
      });

    // $( "#accordion" ).accordion();
    function toggleIcon(e) {
        $(e.target)
            .prev('.panel-heading')
            .find(".more-less")
            .toggleClass('fa-plus fa-minus');
    }
    $('.panel-group').on('hidden.bs.collapse', toggleIcon);
    $('.panel-group').on('shown.bs.collapse', toggleIcon);
})(jQuery);


jQuery('.carousel[data-type="multi"] .item').each(function(){
    var next = jQuery(this).next();
    if (!next.length) {
        next = jQuery(this).siblings(':first');
    }
    next.children(':first-child').clone().appendTo(jQuery(this));

    for (var i=0;i<2;i++) {
        next=next.next();
        if (!next.length) {
            next = jQuery(this).siblings(':first');
        }

        next.children(':first-child').clone().appendTo(jQuery(this));
    }
});


function myFunction() {
    document.getElementById("myDropdown").classList.toggle("show");
  }


function showSearch(){
    jQuery(".navbar-toggler").hide();
    jQuery("#search-mb-box").hide();
    jQuery(".mobile-logo").hide();
    jQuery("#product-search-0").toggle();
    jQuery(".back-arrow").show();
    jQuery("#account_mob_icon").hide();
    jQuery("#cart_mob_icon").hide();
    jQuery('#showSearch').addClass('productSearch')	;
    document.getElementById("product-search-field-0").focus();
}


	jQuery(function($) {

		 	$(".back-arrow").on('click',function() {
                $("#product-search-0").toggle();
                jQuery("#search-mb-box").show();
			 	$(".navbar-toggler").show();
				$(".mobile-logo").show();
 				$(".back-arrow").hide();
				$("#account_mob_icon").show();
                $("#cart_mob_icon").show();
                jQuery('#showSearch').removeClass('productSearch')	;
             });


       $( ".mega-menu-item-127651" ).after( function(e) {
    //         // Handler for .load() called.
            e.preventDefault;
    //         console.log($(this));
    //         var obj = $(this).find('#product-search-field-2');
    //         obj.focus();
            window.setTimeout(function () {
                const productSearchField = document.getElementById("product-search-field-1");

                if (productSearchField) {
                    productSearchField.focus();
                }

            }, 0);
        });

		if (window.location.hash) {
            var hash = window.location.hash;

            if ($(hash).length) {
                $('.single-product '+ hash).collapse({
                    toggle: true
                });
            }
        }
    });






/**
 * Custom JavaScript Functionality.
 */
const application = {
    data: {
        collapseGroups: 10,
    },

    initialize: function() {
        this.bindTriggers();
        this.focusCollapseGroup(1);
        this.fixNavToggler();

        function toggleDisplay(element) {
            if (element.style.display === "none" || element.style.display === "") {
                element.style.display = "block";
            } else {
                element.style.display = "none";
            }
        }
        
        if (!document.querySelector('.postid-8104')) {
            document.querySelector(".navbar-toggler").addEventListener("click", function() {
                toggleDisplay(document.querySelector(".mobile-logo"));
                toggleDisplay(document.querySelector(".nav-search"));
                toggleDisplay(document.querySelector(".mobile-menu-user"));
            });
        }
    },

    bindTriggers: function() {
        for(let x = 0; x < this.data.collapseGroups; x++) {
            const collapseGroupId = (x + 1);
            const selector = `.focus-collapse-group-${collapseGroupId}`;
            const elements = document.querySelectorAll(selector);

            elements.forEach((element) => {
                element.addEventListener("click", () => {
                    this.focusCollapseGroup(collapseGroupId);
                });
            });
        }
    },

    focusCollapseGroup: function(focusCollapseGroupId) {
        for(let x = 0; x < this.data.collapseGroups; x++) {
            const collapseGroupId = (x + 1);

            if (collapseGroupId == focusCollapseGroupId) {
                jQuery(`.collapse-group-${collapseGroupId}`).collapse("show");
            } else {
                jQuery(`.collapse-group-${collapseGroupId}`).collapse("hide");
            }
        }
    },

    fixNavToggler: function () {
        // Initialize.
        jQuery('.postid-8104 .navbar-toggler').each(function () {
            console.log('Initialize nav fix.');
            const el = this;

            jQuery(el).click();

            if (jQuery(el).data('target')) {
                const targetSelector = jQuery(el).data('target');

                jQuery(el).addClass('collapsed');
                jQuery(targetSelector)
                    .addClass('collapse')
                    .removeClass('collapsing show')
                    .css('height', '');
                jQuery(".mobile-logo").show();
                jQuery(".nav-search ").show();
                jQuery(".mobile-menu-user").hide();
            }
        });

        // Add event listener.
        jQuery('.postid-8104 .navbar-toggler').click(function() {
            const el = this;

            if (jQuery(el).data('target')) {
                const targetSelector = jQuery(el).data('target');

                jQuery(targetSelector).removeClass('collapsing').toggleClass('show');
                jQuery(el).toggleClass('collapsed');
                jQuery(".mobile-logo").toggle();
                jQuery(".nav-search ").toggle();
                jQuery(".mobile-menu-user").toggle();
            }
        });
    }
};

addEventListener('DOMContentLoaded', (event) => {
    application.initialize();
});



