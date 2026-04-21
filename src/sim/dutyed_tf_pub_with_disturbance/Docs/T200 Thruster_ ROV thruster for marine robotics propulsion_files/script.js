; (function ($) {

	// Check video format.
	function isVideo(url) {
		var regex = /^(https?:\/\/)?((www\.)?youtube\.com|vimeo\.com|dailymotion\.com|wistia\.com|hubspot\.com|sproutvideo\.com|hippovideo\.io|brightcove\.com|spotlightr\.com|vidyard\.com)\/.+$/i;
		return regex.test(url);
	}
	$(document).find('.sp-easy-accordion').each(function () {
		var accordion_id = $(this).attr('id'),
			_this = $(this),
			ex_icon = _this.data('ex-icon'),
			keep_accordion = _this.data('keep-accordion'),
			col_icon = _this.data('col-icon'),
			ea_active = _this.data('ea-active'),
			ea_mode = _this.data('ea-mode'),
			multiColumn = _this.data('ea-multi-column'),
			preloader = _this.data('preloader'),
			autoplaytime = _this.data('autoplaytime'),
			autoclose = _this.data('autoclose'),
			expand_collapse = _this.data('expand'),
			scroll_active_item = _this.data('scroll-active-item'),
			offset_to_scroll = _this.data('offset-to-scroll');
		var eap_collapse_button_id = $(this).parents('.sp-eap-container').attr('id');
		var multiColumnAccordion = multiColumn === ' ea-multi-column' ? '.eap-multi-items-container >' : '';
		if (' ea-multi-column' === multiColumn && 'ea-auto' === ea_active) {
			ea_active = 'ea-hover';
		}

		$(document).ready(function () {

			var collapseLink = $("#" + eap_collapse_button_id + " .eap_faq_collapse_button a"),
				collapseText = $(collapseLink).parent('.eap_faq_collapse_button').data('collapse-text'),
				expandText = $(collapseLink).parent('.eap_faq_collapse_button').data('expand-text'),
				numPanelOpen = $("#" + eap_collapse_button_id + '> .ea-expand').length;

			if (numPanelOpen > 1) {
				$(collapseLink).html(`${collapseText} <span><i class="fa fa-angle-down"></i></span> <span><i class="fa fa-angle-up"></i></span>`);
			} else {
				$(collapseLink).addClass("active");
				$(collapseLink).html(`${expandText} <span><i class="fa fa-angle-down"></i></span> <span><i class="fa fa-angle-up"></i></span>`);
			}
		});

		if (!$('body').attr('notLoadAfterAjaxRequest')) {
			// Collapse/Expand button.
			$("#" + eap_collapse_button_id + " .eap_faq_collapse_button > a").on("click", function (e) {
				e.preventDefault();
				var numPanelOpen = $("#" + accordion_id + '> ' + multiColumnAccordion + ' .ea-expand').length;
				numPanelItem = $("#" + accordion_id + '> .ea-card').length,
					collapseText = $(this).parent('.eap_faq_collapse_button').data('collapse-text'),
					expandText = $(this).parent('.eap_faq_collapse_button').data('expand-text');
				if (numPanelOpen == 0 || numPanelOpen < numPanelItem) {
					$(this).addClass("active");
				}
				if ($(this).hasClass('active')) {
					$(this).html(`${collapseText} <span><i class="fa fa-angle-down"></i></span> <span><i class="fa fa-angle-up"></i></span>`);
					$("#" + accordion_id + " > " + multiColumnAccordion + " .ea-card > .sp-collapse").spcollapse("show");
					$(this).removeClass("active");
				} else {
					$(this).addClass("active");
					$(this).html(`${expandText} <span><i class="fa fa-angle-down"></i></span> <span><i class="fa fa-angle-up"></i></span>`);
					$("#" + accordion_id + " > " + multiColumnAccordion + " .ea-card > .sp-collapse").spcollapse("hide");
				}
			})
		}

		function activeEvents() {
			$("#" + accordion_id + " > .ea-card > .sp-collapse").on("hide.bs.spcollapse", function (e) {
				$(this).parent(".ea-card").removeClass("ea-expand");
				$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(col_icon).removeClass(ex_icon);
				e.stopPropagation();
			});
			$("#" + accordion_id + " > .ea-card > .sp-collapse").on("show.bs.spcollapse", function (e) {
				$(this).parent(".ea-card").addClass("ea-expand");
				$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(ex_icon).removeClass(col_icon);
				e.stopPropagation();
			});

			if (ea_mode === 'vertical') {
				if (ea_active === 'ea-click') {
					$("#" + accordion_id).each(function () {
						$("#" + accordion_id + " > .ea-card > .ea-header").on("click", function () {
							$("#" + accordion_id + " > .ea-card > .sp-collapse").on("hide.bs.spcollapse", function (e) {
								$(this).parent(".ea-card").removeClass("ea-expand");
								$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(col_icon).removeClass(ex_icon);
								e.stopPropagation();
							})
							$("#" + accordion_id + " > .ea-card > .sp-collapse").on("show.bs.spcollapse", function (e) {
								$(this).parent(".ea-card").addClass("ea-expand");
								$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(ex_icon).removeClass(col_icon);
								e.stopPropagation();
							})
						});
					});
				};
				if (ea_active === 'ea-auto') {

					$("#" + accordion_id).each(function () {
						$("#" + accordion_id + " > .ea-card > .ea-header").on("click", function () {
							$("#" + accordion_id + " > .ea-card > .sp-collapse").on("hide.bs.spcollapse", function (e) {
								$(this).parent(".ea-card").removeClass("ea-expand");
								$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(col_icon).removeClass(ex_icon);
								e.stopPropagation();
							})
							$("#" + accordion_id + " > .ea-card > .sp-collapse").on("show.bs.spcollapse", function (e) {
								$(this).parent(".ea-card").addClass("ea-expand");
								$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(ex_icon).removeClass(col_icon);
								e.stopPropagation();
							})
						});
					});

					function ea_autoplay() {
						var nextItem = $("#" + accordion_id + " > .sp-ea-single.ea-expand").next();

						while (nextItem.length && nextItem.hasClass('eap_inactive')) {
							nextItem = nextItem.next();
						}

						if (expand_collapse) {
							$("#" + accordion_id + " > .ea-card > .sp-collapse").spcollapse("hide");
						}
						if (!nextItem.length) {
							nextItem = $("#" + accordion_id + " > .ea-card").not('.eap_inactive').first();
							if (!nextItem.length) {
								return;
							}
						}

						$(nextItem).find('.sp-collapse').spcollapse("show");
					}
					if (!$('body').attr('notLoadAfterAjaxRequest')) {
						var interval = setInterval(ea_autoplay, autoplaytime);
						$("#" + accordion_id + ".sp-easy-accordion").hover(
							function () {
								clearInterval(interval);
							},
							function () {
								interval = setInterval(ea_autoplay, autoplaytime);
							}
						);
					}
				}

				if (ea_active === 'ea-hover') {
					if (autoclose == '1') {
						$("#" + accordion_id + " > .ea-card")
							.on("mouseenter", function () {
								$(this).children(".sp-collapse").spcollapse("show");
							})
							.on("mouseleave", function () {
								$(this).children(".sp-collapse").spcollapse("hide");
							});
					} else {
						$("#" + accordion_id + " > .ea-card").on('mouseover', function () {
							$(this).children(".sp-collapse").spcollapse("show");
						});
					}
					$("#" + accordion_id + " > .ea-card > .sp-collapse").on("hide.bs.spcollapse", function (e) {
						$(this).parent(".ea-card").removeClass("ea-expand");
						$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(col_icon).removeClass(ex_icon);
						e.stopPropagation();
					})
					$("#" + accordion_id + " > .ea-card > .sp-collapse").on("show.bs.spcollapse", function (e) {
						$(this).parent(".ea-card").addClass("ea-expand");
						$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(ex_icon).removeClass(col_icon);
						e.stopPropagation();
					})
				};

				// Scroll to active item scripts.
				if (scroll_active_item) {
					$("#" + accordion_id + ' .sp-collapse').on('show.bs.spcollapse', function (e) {
						var $panel = $(this).closest('.ea-card');
						setTimeout(function (e) {
							$('html,body').animate({
								scrollTop: $panel.offset().top - offset_to_scroll
							}, 500);
						}, 500)
					});
				}
			} else if (ea_mode === 'horizontal') {
				var sp_selector = $("#" + accordion_id + ".sp-horizontal-accordion > .single-horizontal:not(.ea-expand)");
				// Horizontal scrollbar scripts.
				var singleItemWidth = sp_selector.outerWidth();
				var horizontalItemsWidth = sp_selector.length * (singleItemWidth + parseInt(sp_selector.css("margin-right"))) + 400;
				var wrapperWidth = $("#" + accordion_id + ".sp-horizontal-accordion").parents('.sp-eap-container').outerWidth();
				var scrollableWidth = horizontalItemsWidth > wrapperWidth ? horizontalItemsWidth : wrapperWidth;
				$("#" + accordion_id + ".sp-horizontal-accordion").css("width", scrollableWidth);
				if (horizontalItemsWidth > wrapperWidth) {
					$("#" + accordion_id + ".sp-horizontal-accordion").wrap("<div class='sp-horizontal-accordion-wrapper'></div>");
				}
				var count = sp_selector.length,
					itemMargin = count > 1 ? parseInt(sp_selector.css("margin-right")) * count : parseInt($("#" + accordion_id + ".sp-horizontal-accordion > .single-horizontal.ea-expand").css("margin-right")),
					item_width = count > 1 ? (sp_selector.outerWidth() * count) + itemMargin : parseInt($("#" + accordion_id + ".sp-horizontal-accordion > .single-horizontal.ea-expand").outerWidth()) + itemMargin,
					containerWidth = $("#" + accordion_id + ".sp-horizontal-accordion").outerWidth(),
					activeWidth = containerWidth - item_width,
					activeWidth = activeWidth >= 400 ? activeWidth : 400,
					activeItem = $("#" + accordion_id + ".sp-horizontal-accordion > .single-horizontal.ea-expand");
				activeItem.addClass("active").css("width", activeWidth);
				if (ea_active === 'ea-hover') {
					$("#" + accordion_id + " > .ea-card > .ea-header").hover(function (e) {
						$(this).siblings(".sp-collapse").spcollapse("toggle");
						e.stopPropagation();
					});
				} else {
					$("#" + accordion_id + " > .ea-card > .ea-header").on('click', function (e) {
						$(this).siblings(".sp-collapse").spcollapse("toggle");
						e.stopPropagation();
					});
				}
				$("#" + accordion_id + " > .ea-card > .sp-collapse").on("show.bs.spcollapse", function (e) {
					$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(ex_icon).removeClass(col_icon);
					$(this).parent(".ea-card").addClass("ea-expand").css("width", activeWidth);
					e.stopPropagation();
				});
				$("#" + accordion_id + " > .ea-card > .sp-collapse").on("hide.bs.spcollapse", function (e) {
					var contentWidth = $(this).outerWidth() >= 400 ? $(this).outerWidth() : 400;
					$(this).parent(".ea-card").removeClass("ea-expand").css("width", "");
					$(this).css("width", contentWidth);
					$(this).siblings(".ea-header").find(".ea-expand-icon").addClass(col_icon).removeClass(ex_icon);
					e.stopPropagation();
				});
				if (ea_active === 'ea-auto') {
					function ea_autoplay() {
						var nextItem = $("#" + accordion_id + " > .ea-card.ea-expand").next();
						if (!nextItem.length) {
							$("#" + accordion_id + " > .ea-card > .ea-header a")[0].click();
						}
						$(nextItem).find('.ea-header a').trigger('click');
					}
					if (!$('body').attr('notLoadAfterAjaxRequest')) {
						var interval = setInterval(ea_autoplay, autoplaytime); // Replace 3000 for delay between each slide.
						$("#" + accordion_id + " > .ea-card").hover(function () {
							clearInterval(interval);
						}, function () {
							interval = setInterval(ea_autoplay, autoplaytime); // Replace 3000 for delay between each slide.
						});
					}
				}
			}
			if ($('.ea-youtube-wrapper').length < 1) {
				$(".ea-body object,.ea-body embed").wrap("<div class='ea-youtube-wrapper'></div>");
			}
			// Fix of  Twenty twenty theme iframe conflict issue.
			setTimeout(function () {
				$('.ea-body iframe[style*="width: 0px"]').css({ 'width': '', 'height': '' });
			}, 300);
			// Iframe wrapper 
			$('#' + accordion_id + ' iframe:not(.ea-iframe,.skip,[src*="omny.fm/"])').each(function (i) {
				let src = $(this).attr('src');
				// Check if the iframe source is video or not.
				if (isVideo(src)) {
					let max_width = $(this).attr('width') > 100 ? 'max-width:' + $(this).attr('width') + 'px' : '';
					$(this).addClass('ea-iframe').wrap("<div class='ea-iframe-container " + accordion_id + "_" + i + " '></div>");
					if (max_width) {
						$(this).parent('.ea-iframe-container').wrap("<div style='" + max_width + ";width: 100%;display: inline-block;'></div>");
					}
				}
			});
		}
		activeEvents();

		var preloader_id = $('.accordion-preloader').attr('id');
		if (preloader === 1) {
			$("#" + accordion_id).each(function () {
				$('#' + preloader_id).animate({
					opacity: 0,
				}, 500).remove();
				$('#' + accordion_id).find('.ea-card').animate({
					opacity: 1
				}, 500);
			});
		}

		$("a[href*='#collapse']").on('click', function () {
			const theTarget = $(this).prop("hash");
			$(theTarget + '.spcollapse').parents("[id*='collapse']").spcollapse('show');
			$(theTarget + '.spcollapse').spcollapse('show');
		});

		$(".eap_title_to_slug .sp-ea-single a.collapsed").on('click', (e) => {
			e.preventDefault();
			const _this = $(e.currentTarget);
			const theTarget = _this.data("title");

			setTimeout(() => {
				if (_this.hasClass('collapsed')) {
					// Remove hash from URL
					history.pushState("", document.title, window.location.pathname + window.location.search);
				} else {
					// Update hash in URL
					if (window.location.hash === `#${theTarget}`) {
						$(window).trigger('hashchange');
					} else {
						window.location.hash = theTarget;
					}
				}
			}, 600);
		});

		$(function () {
			if (window.location.hash !== '') {
				handleHashChange(window.location.hash);
			}

			// Handle hash changes dynamically.
			$(window).on('hashchange', function () {
				handleHashChange(window.location.hash);
			});
		});

		// Function to handle hash-based actions
		function handleHashChange(hash) {
			const decodedHash = decodeURIComponent(hash);
			if (decodedHash.indexOf('efaq') !== -1) {
				// Find the matching element and expand it.
				const $element = $('[data-title=' + decodedHash.substring(1) + ']');
				$element.parents(".ea-header")
					.siblings('.sp-collapse')
					.spcollapse('show');
			}
			const $spCollapse = $(decodedHash + '.spcollapse');
			$spCollapse.parents("[id*='collapse']").spcollapse('show');
			$spCollapse.spcollapse('show');
		}

		// FAQ search script.
		var searchTerm, panelContainerId;
		// Create a new contains that is case insensitive
		$.expr.pseudos.containsCaseInsensitive = function (n, i, m) {
			return $(n).text().toUpperCase().indexOf(m[1].toUpperCase()) >= 0;
		};
		var Load_more = _this.parents('.sp-eap-container').find('.sp-eap-load-more'),
			button_attr = Load_more.children('button'),
			total_accordion = button_attr.data('total-post');

		$('#eap_faq_search_bar_' + accordion_id).on('keyup', debounce(function () {
			const searchTerm = $(this).val();
			const postId = $(this).data('shortcode-id');
			const search_autocomplete = $(this).data('autocomplete');
			const containerId = `sp-eap-accordion-section-${postId}`;
			const $container = $(this).parents('.sp-eap-container');

			// Preloader and opacity changes
			$container.find('.ea-multi-column .sp-ea-single').css('opacity', '0');
			$container.find(".ea-multi-column .eap-multi-items-container").contents().unwrap();

			$.ajax({
				type: 'POST',
				url: sp_eap_ajax_obj.ajax_url,
				data: {
					action: 'sp_eap_ajax_load_search',
					nonce: sp_eap_ajax_obj.nonce,
					id: postId,
					keyword: searchTerm,
					event: 'search',
				},
				success: function (response) {
					var $data = $(response);
					// Autocomplete suggestion code.
					if (search_autocomplete) {
						initializeAutocomplete(accordion_id, $data);
					}
					updateAccordionContent(postId, containerId, $data, searchTerm);
					// Highlight search term
					$(`#${accordion_id} .ea-card`).css('opacity', 1);
					$(`#sp-ea-${postId}.sp-easy-accordion`).removeHighlight().highlight(searchTerm);
					// Expand the accordion inside a accordion if search found.
					if (ea_mode === 'vertical' || ea_mode === 'multi-column') {
						handleNestedAccordions(containerId, searchTerm);
					}
				}
			});
		}));

		// Global search script.
		// $('#eap_global_faq_search_bar').on('keyup', debounce(function () {
		// 	const searchTerm = $(this).val();
		// 	$('.sp-easy-accordion').each(function () {
		// 		const $accordion = $(this);
		// 		const accordionId = $accordion.attr('id');
		// 		const postId = $accordion.data('post-id');
		// 		const searchAutocomplete = $accordion.data('autocomplete');
		// 		const containerId = `sp-eap-accordion-section-${postId}`;
		// 		const $container = $accordion.parents('.sp-eap-container');

		// 		// Preloader and opacity changes
		// 		$container.find('.ea-multi-column .sp-ea-single').css('opacity', '0');
		// 		$container.find(".ea-multi-column .eap-multi-items-container").contents().unwrap();
		// 		// AJAX Request
		// 		$.ajax({
		// 			type: 'POST',
		// 			url: sp_eap_ajax_obj.ajax_url,
		// 			data: {
		// 				action: 'sp_eap_ajax_load_search',
		// 				nonce: sp_eap_ajax_obj.nonce,
		// 				id: postId,
		// 				keyword: searchTerm,
		// 				event: 'search',
		// 			},
		// 			success: function (response) {
		// 				const $data = $(response);
		// 				// Autocomplete suggestions
		// 				if (searchAutocomplete) {
		// 					initializeAutocomplete(accordionId, $data, true);
		// 				}
		// 				// Update accordion content
		// 				updateAccordionContent(postId, containerId, $data);

		// 				// Highlight search term
		// 				$(`#${accordionId} .ea-card`).css('opacity', 1);
		// 				$(`#sp-ea-${postId}.sp-easy-accordion`).removeHighlight().highlight(searchTerm);

		// 				// Expand matching cards
		// 				if (['vertical', 'multi-column'].includes(ea_mode)) {
		// 					handleNestedAccordions(containerId, searchTerm);
		// 				}
		// 			},
		// 		});
		// 	});
		// }));
		// $('#eap_global_faq_search_bar').on('keyup', debounce(function () {
		// 	const searchTerm = $(this).val();
		// 	// if (!searchTerm.trim()) return;

		// 	const accordions = $('.sp-easy-accordion');
		// 	const postIds = [];
		// 	const accordionMeta = {};

		// 	accordions.each(function () {
		// 		const $accordion = $(this);
		// 		const postId = $accordion.data('post-id');
		// 		const autocomplete = $accordion.data('autocomplete');
		// 		const containerId = `sp-eap-accordion-section-${postId}`;
		// 		const $container = $accordion.closest('.sp-eap-container');

		// 		postIds.push(postId);
		// 		accordionMeta[postId] = {
		// 			accordionId: $accordion.attr('id'),
		// 			autocomplete,
		// 			container: $container
		// 		};

		// 		// Optional: visual feedback before request
		// 		$container.find('.ea-multi-column .sp-ea-single').css('opacity', '0');
		// 		$container.find(".ea-multi-column .eap-multi-items-container").contents().unwrap();
		// 	});

		// 	if (!searchTerm) {
		// 		// 🔁 Reset full content if search is cleared
		// 		$.ajax({
		// 			type: 'POST',
		// 			url: sp_eap_ajax_obj.ajax_url,
		// 			data: {
		// 				action: 'sp_eap_ajax_load_search',
		// 				nonce: sp_eap_ajax_obj.nonce,
		// 				keyword: '',
		// 				ids: postIds,
		// 				event: 'search',
		// 			},
		// 			success: function (response) {
		// 				if (response.success && response.data) {
		// 					$.each(response.data, function (postId, html) {
		// 						const meta = accordionMeta[postId];
		// 						if (!meta) return;

		// 						const $data = $(html);
		// 						updateAccordionContent(postId, `sp-eap-accordion-section-${postId}`, $data);
		// 						$(`#${meta.accordionId} .ea-card`).css('opacity', 1);
		// 					});
		// 				}
		// 			},
		// 			error: function (xhr) {
		// 				console.error('Global reset AJAX error:', xhr);
		// 			}
		// 		});
		// 		return;
		// 	}

		// 	// One combined AJAX call
		// 	$.ajax({
		// 		type: 'POST',
		// 		url: sp_eap_ajax_obj.ajax_url,
		// 		data: {
		// 			action: 'sp_eap_ajax_load_search',
		// 			nonce: sp_eap_ajax_obj.nonce,
		// 			keyword: searchTerm,
		// 			ids: postIds,
		// 			event: 'search',
		// 		},
		// 		success: function (response) {
		// 			if (response.success && response.data) {
		// 				$.each(response.data, function (postId, html) {
		// 					const meta = accordionMeta[postId];
		// 					if (!meta) return;

		// 					const $data = $(html);
		// 					updateAccordionContent(postId, `sp-eap-accordion-section-${postId}`, $data);
		// 					$(`#${meta.accordionId} .ea-card`).css('opacity', 1);
		// 					$(`#sp-ea-${postId}.sp-easy-accordion`).removeHighlight().highlight(searchTerm);

		// 					if (meta.autocomplete) {
		// 						initializeAutocomplete(meta.accordionId, $data, true);
		// 					}

		// 					if (['vertical', 'multi-column'].includes(ea_mode)) {
		// 						handleNestedAccordions(`sp-eap-accordion-section-${postId}`, searchTerm);
		// 					}
		// 				});
		// 			}
		// 		},
		// 		error: function (xhr) {
		// 			console.error('Global search AJAX error:', xhr);
		// 		}
		// 	});
		// }));

		$('#eap_global_faq_search_bar').on('keyup', debounce(function () {
			const $input = $(this);
			const searchTerm = $input.val().trim();
			const isReset = searchTerm === '';

			const postIds = [];
			const accordionMeta = {};

			// Cache meta for each accordion
			$('.sp-easy-accordion').each(function () {
				const $accordion = $(this);
				const postId = $accordion.data('post-id');
				if (!postId) return;

				const meta = {
					accordionId: $accordion.attr('id'),
					autocomplete: $accordion.data('autocomplete'),
					container: $accordion.closest('.sp-eap-container')
				};

				postIds.push(postId);
				accordionMeta[postId] = meta;

				// Visual feedback before AJAX
				meta.container.addClass('loading');
				meta.container.find('.ea-multi-column .sp-ea-single').css('opacity', '0');
				meta.container.find('.ea-multi-column .eap-multi-items-container').contents().unwrap();
			});

			// Prepare AJAX data
			const ajaxData = {
				action: 'sp_eap_ajax_load_search',
				nonce: sp_eap_ajax_obj.nonce,
				keyword: searchTerm,
				ids: postIds,
				event: 'search'
			};

			// AJAX call
			$.ajax({
				type: 'POST',
				url: sp_eap_ajax_obj.ajax_url,
				data: ajaxData,
				success: function (response) {
					if (response.success && response.data) {
						$.each(response.data, function (postId, html) {
							const meta = accordionMeta[postId];
							if (!meta) return;

							const $data = $(html);
							updateAccordionContent(postId, `sp-eap-accordion-section-${postId}`, $data, searchTerm, true);

							const $accordion = $(`#${meta.accordionId}`);
							$accordion.find('.ea-card').css('opacity', 1);
							meta.container.removeClass('loading');

							if (!isReset) {
								$accordion.removeHighlight().highlight(searchTerm);

								if (meta.autocomplete) {
									initializeAutocomplete(meta.accordionId, $data, true);
								}

								if (['vertical', 'multi-column'].includes(ea_mode)) {
									handleNestedAccordions(`sp-eap-accordion-section-${postId}`, searchTerm);
								}
							}
						});
					}
				}
			});
		}, 500)); // Adjust debounce delay if needed

		// 🔄 Clear button
		$('#sp-global-clear-search-button').on('click', function () {
			$('#eap_global_faq_search_bar').val('').trigger('keyup');
		});


		// Autocomplete initialization
		function initializeAutocomplete(accordionId, $data, $globalSearch = false) {
			const optionTexts = [];
			$data.find('.ea-header').each(function (i, v) {
				JSON.stringify(v);
				optionTexts[i] = $(v).text().trim();
			});
			$.widget('ui.customAutocompleteWidget', $.ui.autocomplete, {
				renderItem: function (ul, item) {
					return $('<li>')
						.append($('<a>').html(decodeURI(item.label)))
						.appendTo(ul);
				}
			});
			const searchInput = $globalSearch ? $('#eap_global_faq_search_bar') : $(`#eap_faq_search_bar_${accordionId}`);
			searchInput.customAutocompleteWidget({
				source: optionTexts,
				delay: 0,
				create: function () {
					$(this).customAutocompleteWidget('widget')
						.addClass('eap-autocomplete-wrapper')
						.css({
							'max-height': 500,
							'overflow-y': 'scroll',
							'overflow-x': 'hidden'
						});
				},
				select: function (event, ui) {
					$(this).val(ui.item.label).trigger('keyup');
					return false;
				}
			});
		}

		function updateAccordionContent(id, containerId, $data, searchTerm = '', globalSearch = '') {
			$(`#sp-ea-${id}`).empty().html($data);
			const $container = $(`#${containerId}`);
			$container.find('.sp-eap-load-more').hide();
			$container.find('.sp-eap-ajax-number-pagination').hide();
			$container.find('.accordion-preloader').fadeOut(100);

			accordion_id = $container.find('.sp-easy-accordion').attr('id'),
				_this = $container.find('.sp-easy-accordion'),
				ex_icon = _this.data('ex-icon'),
				keep_accordion = _this.data('keep-accordion'),
				col_icon = _this.data('col-icon'),
				ea_active = _this.data('ea-active'),
				ea_mode = _this.data('ea-mode'),
				multiColumn = _this.data('ea-multi-column'),
				preloader = _this.data('preloader'),
				autoplaytime = _this.data('autoplaytime'),
				autoclose = _this.data('autoclose'),
				expand_collapse = _this.data('expand'),
				scroll_active_item = _this.data('scroll-active-item'),
				offset_to_scroll = _this.data('offset-to-scroll');
			eap_collapse_button_id = $container.attr('id');
			multiColumnAccordion = multiColumn === ' ea-multi-column' ? '.eap-multi-items-container >' : '';
			if (' ea-multi-column' === multiColumn && 'ea-auto' === ea_active) {
				ea_active = 'ea-hover';
			}
			activeEvents();
			multiColumnAccordionWrapper();
			keepActiveAccordion();

			// Reset pagination if search is empty
			const $searchInputHide = $container.find(`#eap_faq_search_bar_container`);
			if (searchTerm && globalSearch) {
				$searchInputHide.hide();
			} else {
				$searchInputHide.show();
			}

			if (!searchTerm) {
				$container.find('.sp-eap-load-more').show();
				$container.find('.sp-eap-ajax-number-pagination').show();
				$.getScript(sp_eap_ajax_pagi.loadPagiScript);
			}
			const $sectionTitle = $container.find(`.eap_section_title_${id}`);
			if ($container.find('.sp-ea-single').length > 0) {
				$sectionTitle.show();
			} else {
				$sectionTitle.hide();
			}
		}

		// Handle nested accordions during a search
		function handleNestedAccordions(containerId, searchTerm) {
			const $accordion = $(`#${containerId} .sp-easy-accordion`);
			if ($accordion.length > 1) {
				$accordion.find('.ea-card').each(function () {
					const $card = $(this);
					if (!$card.find('.eap-search-highlight').length && searchTerm) {
						$card.remove();
					}
				});
			}
		}

		function debounce(func, wait = 800) {
			let timeout;
			return function (...args) {
				clearTimeout(timeout);
				timeout = setTimeout(() => func.apply(this, args), wait);
			};
		}

		// FAQ search result highlighting script.
		jQuery.fn.highlight = function (pat) {
			function innerHighlight(node, pat) {
				var skip = 0;
				if (node.nodeType == 3) {
					var pos = node.data.toUpperCase().indexOf(pat);
					pos -= (node.data.substr(0, pos).toUpperCase().length - node.data.substr(0, pos).length);
					if (pos >= 0) {
						var spanTag = document.createElement('span');
						spanTag.className = 'eap-search-highlight';
						var middleBit = node.splitText(pos);
						var endBit = middleBit.splitText(pat.length);
						var middleClone = middleBit.cloneNode(true);
						spanTag.appendChild(middleClone);
						middleBit.parentNode.replaceChild(spanTag, middleBit);
						skip = 1;
					}
				}
				else if (node.nodeType == 1 && node.childNodes && !/(script|style)/i.test(node.tagName)) {
					for (var i = 0; i < node.childNodes.length; ++i) {
						i += innerHighlight(node.childNodes[i], pat);
					}
				}
				return skip;
			}
			return this.length && pat && pat.length ? this.each(function () {
				innerHighlight(this, pat.toUpperCase());
			}) : this;
		};
		jQuery.fn.removeHighlight = function () {
			return this.find("span.eap-search-highlight").each(function () {
				this.parentNode.firstChild.nodeName;
				with (this.parentNode) {
					replaceChild(this.firstChild, this);
					normalize();
				}
			}).end();
		};

		// highlight matching word.
		$('#eap_faq_search_bar_' + accordion_id).on('keyup', function () {
			searchTerm = $(this).val();
			$("#" + accordion_id + ".sp-easy-accordion").removeHighlight().highlight(searchTerm);
		});
		// Add event listener to the " Search clear" button
		$('#clear-search-button-' + accordion_id).on('click', function () {
			$('#eap_faq_search_bar_' + accordion_id).val('');
			$('#eap_faq_search_bar_' + accordion_id).trigger('keyup');
		});

		// $('#eap_global_faq_search_bar').on('keyup', function () {
		// 	searchTerm = $(this).val();
		// 	$('.sp-easy-accordion').removeHighlight().highlight(searchTerm);
		// });
		// $('#sp-global-clear-search-button').on('click', function () {
		// 	$('#eap_global_faq_search_bar').val('');
		// 	$('#eap_global_faq_search_bar').trigger('keyup');
		// });

		// Multi Column Accordion scripts.
		function multiColumnAccordionWrapper() {
			const container = $("#" + accordion_id + ".sp-easy-accordion.ea-multi-column");
			// Divide items into two divs
			if (container.length > 0) {
				const items = container.children(".sp-ea-single");
				const halfLength = Math.ceil(items.length / 2);
				const div1 = $("<div>").addClass("eap-multi-items-container").appendTo(container);
				items.slice(0, halfLength).appendTo(div1);

				const div2 = $("<div>").addClass("eap-multi-items-container").appendTo(container);
				items.slice(halfLength).appendTo(div2);
				const checkPagination = container.parents('.sp-eap-container').find('.sp-eap-load-more');
				// Set the opacity of .sp-ea-single elements to 1
				container.find(".eap-multi-items-container .sp-ea-single").css('opacity', '1');
			}
		};
		multiColumnAccordionWrapper();

		// custom keep accordion.
		function keepActiveAccordion() {
			$(document).ready(function () {
				if (keep_accordion) {
					var active_col = keep_accordion;
					$("#" + accordion_id).find(".ea-expand-icon").addClass(col_icon).removeClass(ex_icon);
					if ($("#" + accordion_id).parents('.spcollapse').length > 0) {
						if ($("#" + accordion_id).parents('.spcollapse.show').length > 0) {
							$("#" + accordion_id + " > .ea-card:nth-child(" + active_col + "):not(.ea-expand) > .ea-header a").trigger('click');
						}
					} else {
						$("#" + accordion_id + " > .ea-card:nth-child(" + active_col + "):not(.ea-expand) > .ea-header a").trigger('click');
					}
				}
			});
		}
		keepActiveAccordion();


		// Prevent the default link click behavior.
		$("#" + accordion_id + " > .ea-card .ea-header a").on('click', function (event) {
			event.preventDefault();
		});

		// Initialize the Image Accordion with improved practices
		const initImageAccordion = ($accordion, activatorEvent, autoCloseTabs, animationClass, autoplayDelay, enablePauseOnHover) => {
			const $items = $accordion.find(".sp-eap-image-accordion-item");
			let autoplayInterval;
			let currentItemIndex = 1;

			// Start the autoplay functionality
			const startAutoplay = () => {
				stopAutoplay();
				autoplayInterval = setInterval(() => {
					let $currentItem = $items.eq(currentItemIndex);

					// Skip inactive items
					while ($currentItem.hasClass('eap_inactive_item')) {
						currentItemIndex = (currentItemIndex + 1) % $items.length;
						$currentItem = $items.eq(currentItemIndex);

						// Stop autoplay if all items are inactive
						if (currentItemIndex === 0 && $currentItem.hasClass('eap_inactive_item')) {
							stopAutoplay();
							return;
						}
					}

					// Activate the next valid item
					activateItem($currentItem);
					currentItemIndex = (currentItemIndex + 1) % $items.length;
				}, autoplayDelay);
			};

			// Stop the autoplay functionality
			const stopAutoplay = () => {
				clearInterval(autoplayInterval);
			};

			// Activate an accordion item
			const activateItem = ($item) => {
				$item.addClass('active').siblings().removeClass('active');
				if (animationClass !== 'false') {
					$items.not($item).find('.sp-eap-image-accordion-content-wrapper').removeClass(animationClass);
					$item.find('.sp-eap-image-accordion-content-wrapper').addClass(animationClass);
				}
			};

			// Event binding for hover activation
			if (activatorEvent === 'ea-hover') {
				$accordion.on('mouseover', '.sp-eap-image-accordion-item', (e) => {
					activateItem($(e.currentTarget));
				});
				if (autoCloseTabs) {
					$accordion.on('mouseleave', () => {
						$items.removeClass('active');
					});
				}
			}

			// Event binding for click activation
			if (activatorEvent === 'ea-click' || activatorEvent === 'ea-auto') {
				$accordion.on('click', '.sp-eap-image-accordion-item', (e) => {
					const $clickedItem = $(e.currentTarget);

					// Stop autoplay and activate the clicked item
					stopAutoplay();
					activateItem($clickedItem);
					currentItemIndex = $items.index($clickedItem);

					// Restart autoplay if applicable
					if (activatorEvent === 'ea-auto') {
						startAutoplay();
					}
				});
			}

			// Enable autoplay
			if (activatorEvent === 'ea-auto') {
				startAutoplay();

				// Pause autoplay on hover
				if (enablePauseOnHover) {
					$accordion.on('mouseenter', stopAutoplay);
					$accordion.on('mouseleave', startAutoplay);
				}
			}
		};

		// Initialize the accordion dynamically on document ready
		$(document).ready(() => {
			$(".sp-eap-image-accordion-wrapper").each(function () {
				const $parent = $(this).closest('.eap-image-accordion');
				const activatorEvent = $parent.data('ea-active');
				const autoCloseTabs = $parent.data('autoclose');
				const autoplayDelay = $parent.data('autoplaytime');
				const enablePauseOnHover = $parent.data('pause');
				const animationClass = $parent.data('animation');

				// Initialize each accordion
				initImageAccordion($(this), activatorEvent, autoCloseTabs, animationClass, autoplayDelay, enablePauseOnHover);
			});
		});

		// Product quantity scripts.
		$('.eap-product-quantity .eap_input_text').on('change', function () {
			var new_quantity = $(this).val();
			var selector = $(this).parents('.eap-product-cart-button').find(".product_type_simple.add_to_cart_button");
			selector[0].setAttribute("data-quantity", new_quantity);
		});
	});

	if ($('body').find('.sp-easy-accordion').length > 0) {
		$('body').attr('notLoadAfterAjaxRequest', '1');
	}

	// wpforms initialization.
	$(document).ajaxComplete(function () {
		if (typeof wpforms !== 'undefined') {
			wpforms.init();
		}
	});

})(jQuery);


