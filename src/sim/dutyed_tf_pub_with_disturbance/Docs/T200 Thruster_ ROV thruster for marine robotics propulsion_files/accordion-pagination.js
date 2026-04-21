jQuery(document).ready(function ($) {
	jQuery('body').find('.sp-eap-container').each(function () {
		var accordion_id = $(this).attr('id');
		var pagination_id = accordion_id.match(/[\d\.]+/g);
		jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id + ' button').on('click', function (e) {
			e.preventDefault();
			e.stopPropagation();
			var multiColumnItem = $(this).parents(".sp-eap-container").find('.ea-multi-column .sp-ea-single');
			var multiWrapper = $(this).parents('.sp-eap-container').find(".ea-multi-column .eap-multi-items-container").contents();
			var sid = $(this).data('id');
			var sp_eap_page = $(this).data('page') + 1;
			var sp_eap_total = $(this).data('total');
			var end_text = $(this).parents('.sp-eap-load-more-' + pagination_id).data('text');
			jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id + ' button').attr('sp-eap-processing', 1);
			jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id).before('<div class="sp-eap-infinite-scroll-loader"><svg width="44" height="44" viewBox="0 0 44 44" xmlns="http://www.w3.org/2000/svg" stroke="#444"><g fill="none" fill-rule="evenodd" stroke-width="2"><circle cx="22" cy="22" r="1"><animate attributeName="r" begin="0s" dur="1.8s" values="1; 20" calcMode="spline" keyTimes="0; 1" keySplines="0.165, 0.84, 0.44, 1" repeatCount="indefinite" /> <animate attributeName="stroke-opacity" begin="0s" dur="1.8s" values="1; 0" calcMode="spline" keyTimes="0; 1" keySplines="0.3, 0.61, 0.355, 1" repeatCount="indefinite" /> </circle> <circle cx="22" cy="22" r="1"> <animate attributeName="r" begin="-0.9s" dur="1.8s" values="1; 20" calcMode="spline" keyTimes="0; 1" keySplines="0.165, 0.84, 0.44, 1" repeatCount="indefinite" /> <animate attributeName="stroke-opacity" begin="-0.9s" dur="1.8s" values="1; 0" calcMode="spline" keyTimes="0; 1" keySplines="0.3, 0.61, 0.355, 1" repeatCount="indefinite"/></circle></g></svg></div>');
			$.ajax({
				type: 'POST',
				url: sp_eap_ajax_obj.ajax_url,
				nonce: sp_eap_ajax_obj.nonce,
				data: {
					id: sid,
					nonce: sp_eap_ajax_obj.nonce,
					action: 'sp_eap_ajax_load_search',
					sp_eap_page: sp_eap_page,
					event: 'pagination',
				},
				success: function (response) {
					multiColumnItem.css('opacity', '0');
					multiWrapper.unwrap();
					jQuery('#sp-ea-' + sid).append(response);
					jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id + ' button').data('page', sp_eap_page);
					if (sp_eap_total <= sp_eap_page) {
						jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id).html(end_text);
					}
					jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id + ' button').attr('sp-eap-processing', 0);
					jQuery('#' + accordion_id).find('.sp-eap-infinite-scroll-loader').remove();
					jQuery.getScript(sp_eap_ajax_obj.loadScript);
				}
			})
			e.stopImmediatePropagation();
		});
		// Ajax Infinite Scroll Pagination.
		if (jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id).data('pagi') == 'ajax_infinite_scrl') {
			var bufferBefore = Math.abs(20);
			jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id).hide();
			$(window).scroll(function () {
				if (jQuery('#' + accordion_id).find('.sp-easy-accordion').length) {
					var TopAndContent = jQuery('#' + accordion_id).find('.sp-easy-accordion').offset().top + jQuery('#' + accordion_id).find('.sp-easy-accordion').outerHeight();
					var areaLeft = TopAndContent - $(window).scrollTop()
					if (areaLeft - bufferBefore < $(window).height()) {
						if (jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id + ' button').attr('sp-eap-processing') == 0) {
							jQuery('#' + accordion_id).find('.sp-eap-load-more-' + pagination_id + ' button').trigger('click');
						}
					}
				}
			})
		}
		// Ajax Number Pagination.
		if (jQuery('#' + accordion_id).find('.sp-eap-ajax-number-pagination-' + pagination_id).data('pagination') == 'ajax_number') {
			jQuery('#' + accordion_id).find('.sp-eap-pagination-number a:nth-child(2)').addClass('active');
			$('.prev', '#' + accordion_id).hide();
			jQuery('#' + accordion_id).find('.sp-eap-pagination-number a').on('click', function (e) {
				e.preventDefault();
				e.stopPropagation();
				e.stopImmediatePropagation();
				var _this = $(this);
				var sid = $(this).parents('.sp-eap-pagination-number').data('id');
				var sp_eap_total = $(this).parents('.sp-eap-pagination-number').data('total');
				var sp_eap_page = $(this).data('page') - 1;
				if (_this.hasClass('next')) {
					var sp_eap_page = jQuery('#' + accordion_id).find('.sp-eap-pagination-number a.active:not(.next, .prev)').data('page') - 1;
					sp_eap_page++;
				}
				if (_this.hasClass('prev')) {
					var sp_eap_page = jQuery('#' + accordion_id).find('.sp-eap-pagination-number a.active:not(.next, .prev)').data('page') - 1;
					sp_eap_page--;
				}
				$.ajax({
					type: 'POST',
					url: sp_eap_ajax_obj.ajax_url,
					nonce: sp_eap_ajax_obj.nonce,
					data: {
						id: sid,
						nonce: sp_eap_ajax_obj.nonce,
						action: 'sp_eap_ajax_load_search',
						sp_eap_page: sp_eap_page + 1,
						event: 'search',
					},
					success: function (response) {
						jQuery('#' + accordion_id).find('.sp-easy-accordion').html(response);
						jQuery('#' + accordion_id).find('.sp-eap-pagination-number a').removeClass('active');
						sp_eap_page++;
						$('.sp-eap-page-numbers', '#' + accordion_id).each(function () {
							jQuery('#' + accordion_id).find('.sp-eap-pagination-number a[data-page=' + sp_eap_page + ']').addClass('active');
						})
						$('.sp-eap-pagination-number a.active', '#' + accordion_id).each(function () {
							if (parseInt($(this).data('page')) === Math.ceil(sp_eap_total)) {
								$('.next', '#' + accordion_id).hide();
							} else {
								$('.next', '#' + accordion_id).show();
							}
							if (jQuery('#' + accordion_id).find('.sp-eap-pagination-number a:nth-child(2)').hasClass('active')) {
								$('.prev', '#' + accordion_id).hide();
							} else {
								$('.prev', '#' + accordion_id).show();
							}
						})
						jQuery.getScript(sp_eap_ajax_obj.loadScript);
					}
				})
			})
		}
	});
});